package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.OdometryConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.vision.HopperAlignmentController;
import frc.robot.vision.Limelight.TargetMeasurement;

/** Centers the camera on the selected hopper tag, then verifies it with fresh captures. */
public final class HopperTagAlignmentCommand extends Command {
    public interface IO {
        Pose2d getPose();
        ChassisSpeeds getRobotRelativeSpeeds();
        void resetOdometry(Pose2d pose);
        void driveRobotRelative(ChassisSpeeds speeds);
        void stopMotors();
        void beginHopperTagAlignment(int tagId);
        void endHopperTagAlignment();
        Optional<TargetMeasurement> getNewHopperTagPose(int tagId);
        void fuseHopperTagPose(Pose2d pose, double timestampSeconds);
        void reportHopperTagAlignment(String status, boolean success, double rightMeters);
    }

    private final IO m_io;
    private final Pose2d m_goal;
    private final int m_tagId;
    private final DoubleSupplier m_clock;
    private final BooleanSupplier m_visionEnabled;
    private Pose2d m_cameraCenteredGoal;
    private Translation2d m_goalSum;
    private HopperAlignmentController m_controller;
    private Translation2d m_sum;
    private Translation2d m_clusterFirst;
    private int m_frames;
    private double m_headingSin;
    private double m_headingCos;
    private int m_confirmations;
    private double m_started;
    private double m_captureCutoff;
    private double m_lastCapture;
    private double m_lastRight;
    private boolean m_finished;
    private boolean m_success;
    private boolean m_sawUnreliableHeading;
    private String m_status;

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled) {
        m_io = io;
        m_goal = goal;
        m_tagId = tagId;
        m_clock = clock;
        m_visionEnabled = visionEnabled;
        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        m_started = m_clock.getAsDouble();
        m_captureCutoff = m_started;
        m_lastCapture = Double.NEGATIVE_INFINITY;
        m_lastRight = Double.NaN;
        m_controller = null;
        m_cameraCenteredGoal = m_goal;
        clearCluster();
        m_confirmations = 0;
        m_finished = false;
        m_success = false;
        m_sawUnreliableHeading = false;
        m_io.beginHopperTagAlignment(m_tagId);
        m_io.stopMotors();
        report("waiting for tag " + m_tagId);
    }

    @Override
    public void execute() {
        double now = m_clock.getAsDouble();
        if (!m_visionEnabled.getAsBoolean()) {
            finish(false, "alignment failed: vision disabled");
            return;
        }
        if (now - m_started >= kHopperAlignTimeoutSeconds) {
            finish(false, "alignment timed out: tag " + m_tagId + " center not confirmed");
            return;
        }
        if (now - m_lastCapture > kMaxVisionAgeSeconds) {
            m_confirmations = 0;
        }
        ChassisSpeeds speeds = m_io.getRobotRelativeSpeeds();
        boolean stopped = Math.abs(speeds.vxMetersPerSecond) <= kDriveStraightSpeedToleranceMetersPerSecond
            && Math.abs(speeds.omegaRadiansPerSecond) <= kHopperAlignTurnSpeedToleranceRadPerSec;
        if (m_controller == null && !stopped) {
            clearCluster();
            m_captureCutoff = now;
            m_io.stopMotors();
            report("settling before acquiring tag " + m_tagId);
            return;
        }

        Optional<TargetMeasurement> sample = m_io.getNewHopperTagPose(m_tagId);
        if (sample.isPresent()) {
            TargetMeasurement measurement = sample.get();
            double age = now - measurement.timestampSeconds();
            if (measurement.tagId() != m_tagId || measurement.timestampSeconds() <= m_captureCutoff
                    || measurement.timestampSeconds() <= m_lastCapture
                    || !Double.isFinite(age) || age > kMaxVisionAgeSeconds || age < -kVisionFutureToleranceSeconds
                    || !Double.isFinite(measurement.rightMeters()) || !Double.isFinite(measurement.outwardMeters())
                    || !Double.isFinite(measurement.cameraRightMeters()) || !Double.isFinite(measurement.cameraOutwardMeters())
                    || Math.abs(measurement.rightMeters()) > kHopperTagMaxLateralMeters
                    || measurement.outwardMeters() <= 0.1 || measurement.outwardMeters() > kHopperTagMaxRangeMeters) {
            } else {
                double previousCapture = m_lastCapture;
                m_lastCapture = measurement.timestampSeconds();
                m_lastRight = measurement.cameraRightMeters();
                report(m_status);
                Pose2d directPose = fieldPoseFromTag(measurement, m_goal, m_io.getPose());
                if (m_controller == null) {
                    if (!Double.isFinite(measurement.fieldHeadingDegrees())) {
                        m_sawUnreliableHeading = true;
                        clearCluster();
                        m_io.stopMotors();
                        report("waiting for reliable tag heading for tag " + m_tagId);
                        if (now - m_started >= kHopperTagAcquireTimeoutSeconds) {
                            finish(false, "alignment failed: no reliable tag heading for tag " + m_tagId);
                        }
                        return;
                    }
                    Translation2d position = directPose.getTranslation();
                    double headingRadians = Math.toRadians(measurement.fieldHeadingDegrees());
                    boolean headingDisagrees = m_frames > 0 && Math.abs(MathUtil.angleModulus(
                        headingRadians - Math.atan2(m_headingSin, m_headingCos))) > Math.toRadians(10.0);
                    if (m_clusterFirst == null || m_lastCapture - previousCapture > kMaxVisionAgeSeconds
                            || position.getDistance(m_clusterFirst) > kHopperTagMaxSpreadMeters || headingDisagrees) {
                        clearCluster();
                        m_clusterFirst = position;
                    }
                    m_sum = m_sum.plus(position);
                    m_goalSum = m_goalSum.plus(cameraCenteredGoal(measurement, m_goal).getTranslation());
                    m_headingSin += Math.sin(headingRadians);
                    m_headingCos += Math.cos(headingRadians);
                    m_frames++;
                    if (m_frames >= kHopperTagAcquireFrames) {
                        Pose2d corrected = new Pose2d(m_sum.div(m_frames),
                            new Rotation2d(m_headingCos, m_headingSin));
                        m_io.resetOdometry(corrected);
                        m_cameraCenteredGoal = new Pose2d(m_goalSum.div(m_frames), m_goal.getRotation());
                        // Leave room for small heading errors moving an offset camera sideways.
                        m_controller = new HopperAlignmentController(m_cameraCenteredGoal, kHopperAlignToleranceMeters / 2.0);
                        m_controller.reset(corrected);
                        report("moving onto tag " + m_tagId + " center");
                    }
                } else {
                    // Fuse at capture time to account for movement during camera latency.
                    m_io.fuseHopperTagPose(directPose, measurement.timestampSeconds());
                    boolean centered = Math.abs(measurement.cameraRightMeters()) <= kHopperAlignToleranceMeters
                        && Math.abs(measurement.outwardMeters() - kHopperTagStandOffMeters) <= kHopperAlignToleranceMeters
                        && Math.abs(m_goal.getRotation().minus(m_io.getPose().getRotation()).getRadians())
                            <= kHopperAlignHeadingToleranceRadians;
                    m_confirmations = centered && stopped ? m_confirmations + 1 : 0;
                    if (m_confirmations >= kHopperTagConfirmFrames) {
                        finish(true, "aligned on tag " + m_tagId);
                        return;
                    }
                    if (m_controller.isFinished() && stopped && !centered) {
                        // Odometry reaching its goal is insufficient: the visible tag must agree.
                        m_io.resetOdometry(directPose);
                        m_controller.reset(directPose);
                    }
                }
            }
        }

        if (m_controller == null) {
            m_io.stopMotors();
            if (now - m_started >= kHopperTagAcquireTimeoutSeconds) {
                finish(false, m_sawUnreliableHeading
                    ? "alignment failed: no reliable tag heading for tag " + m_tagId
                    : "alignment failed: no fresh consistent tag " + m_tagId + " pose");
            }
        } else {
            m_io.driveRobotRelative(m_controller.calculate(m_io.getPose(), speeds));
            if (m_controller.isFinished()) {
                report("waiting to confirm tag " + m_tagId + " center");
            }
        }
    }

    /** Convert Limelight's right/outward tag coordinates to WPILib blue-origin field position. */
    public static Pose2d fieldPoseFromTag(TargetMeasurement measurement, Pose2d goal, Pose2d current) {
        Translation2d relative = new Translation2d(
            measurement.outwardMeters() - kHopperTagStandOffMeters, measurement.rightMeters());
        return new Pose2d(goal.getTranslation().plus(relative.rotateBy(goal.getRotation())), current.getRotation());
    }

    /** Shift the robot goal for the measured camera mount offset while preserving bumper clearance. */
    public static Pose2d cameraCenteredGoal(TargetMeasurement measurement, Pose2d nominalGoal) {
        Translation2d robotToCamera = new Translation2d(
            measurement.cameraOutwardMeters() - measurement.outwardMeters(),
            measurement.cameraRightMeters() - measurement.rightMeters());
        // Rotate the measured mount offset to where it will be at the final robot heading.
        Translation2d offsetAtGoal = robotToCamera.rotateBy(nominalGoal.getRotation()
            .minus(Rotation2d.fromDegrees(measurement.fieldHeadingDegrees())));
        Translation2d shift = new Translation2d(0.0, -offsetAtGoal.getY()).rotateBy(nominalGoal.getRotation());
        return new Pose2d(nominalGoal.getTranslation().plus(shift), nominalGoal.getRotation());
    }

    private void clearCluster() {
        m_sum = Translation2d.kZero;
        m_goalSum = Translation2d.kZero;
        m_clusterFirst = null;
        m_frames = 0;
        m_headingSin = 0.0;
        m_headingCos = 0.0;
    }

    private void report(String status) {
        m_status = status;
        m_io.reportHopperTagAlignment(status, m_success, m_lastRight);
    }

    private void finish(boolean success, String status) {
        m_success = success;
        m_finished = true;
        m_io.stopMotors();
        report(status);
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_io.stopMotors();
        m_io.endHopperTagAlignment();
        if (interrupted) {
            m_success = false;
            report("tag alignment interrupted");
        } else {
            report(m_status);
        }
    }
}
