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
import frc.robot.vision.HopperStartView.Reference;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.TargetMeasurement;

/** Returns the robot to its startup position and heading using both hopper tags. */
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
        default void fuseHopperPairPose(Pose2d pose, double timestampSeconds) {
            fuseHopperTagPose(pose, timestampSeconds);
        }
        void reportHopperTagAlignment(String status, boolean success, double rightMeters);
        default void reportHopperImageError(double degrees) {}
        default void reportHopperCompanionImageError(double degrees) {}
        default void reportHopperLeftTarget(double meters) {}
        default void reportHopperStartViewTarget(Reference reference) {}
        default void onHopperPoseAcquired() {}
    }

    private final IO m_io;
    private final Pose2d m_goal;
    private final int m_tagId;
    private final DoubleSupplier m_clock;
    private final BooleanSupplier m_visionEnabled;
    private final double m_maxSpeed;
    private final Reference m_startView;
    private final String m_pairName;
    private HopperAlignmentController m_controller;
    private Translation2d m_sum;
    private Translation2d m_clusterFirst;
    private int m_frames;
    private double m_headingSin;
    private double m_headingCos;
    private int m_confirmations;
    private double m_started;
    private double m_acquireStarted;
    private double m_captureCutoff;
    private double m_lastCapture;
    private double m_lastRight;
    private double m_lastOutwardError;
    private boolean m_finished;
    private boolean m_success;
    private String m_status;

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled) {
        this(io, requirement, goal, tagId, clock, visionEnabled, kHopperAlignMaxSpeedMetersPerSecond);
    }

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled, double maxSpeed) {
        this(io, requirement, goal, tagId, clock, visionEnabled, maxSpeed, null);
    }

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled, double maxSpeed, Reference startView) {
        m_io = io;
        m_startView = startView != null && startView.tagId() == tagId ? startView : null;
        m_goal = m_startView != null ? m_startView.startingPose() : goal;
        m_tagId = tagId;
        m_pairName = Limelight.companionTagId(tagId) + " + " + tagId;
        m_clock = clock;
        m_visionEnabled = visionEnabled;
        m_maxSpeed = maxSpeed;
        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        m_started = m_clock.getAsDouble();
        m_acquireStarted = m_started;
        m_captureCutoff = m_started;
        m_lastCapture = Double.NEGATIVE_INFINITY;
        m_lastRight = Double.NaN;
        m_lastOutwardError = Double.NaN;
        m_controller = null;
        clearCluster();
        m_confirmations = 0;
        m_finished = false;
        m_success = false;
        m_io.beginHopperTagAlignment(m_tagId);
        m_io.reportHopperLeftTarget(0.0);
        if (m_startView != null) {
            m_io.reportHopperStartViewTarget(m_startView);
        }
        m_io.stopMotors();
        report("waiting for hopper tags " + m_pairName);
    }

    @Override
    public void execute() {
        double now = m_clock.getAsDouble();
        if (!m_visionEnabled.getAsBoolean()) {
            finish(false, "alignment failed: vision disabled");
            return;
        }
        if (now - m_started >= (m_startView != null ? kHopperReturnAlignTimeoutSeconds : kHopperAlignTimeoutSeconds)) {
            finish(false, "alignment timed out: hopper position and heading not restored using tags " + m_pairName);
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
            m_acquireStarted = now;
            m_io.driveRobotRelative(new ChassisSpeeds());
            report("settling before acquiring hopper tags " + m_pairName);
            return;
        }

        Optional<TargetMeasurement> sample = m_io.getNewHopperTagPose(m_tagId);
        if (sample.isPresent() && valid(sample.get(), now)) {
            TargetMeasurement measurement = sample.get();
            double previousCapture = m_lastCapture;
            m_lastCapture = measurement.timestampSeconds();
            Pose2d rawPose = measurement.toFieldPose(m_goal);
            // Compare independent two-tag poses in the startup frame. This measures yaw
            // separately from sideways displacement and cancels a fixed startup pose offset.
            Pose2d directPose = m_startView == null ? rawPose : new Pose2d(
                m_startView.startingPose().getTranslation().plus(rawPose.getTranslation()
                    .minus(m_startView.pairedFieldPose().getTranslation())),
                m_startView.startingPose().getRotation().plus(rawPose.getRotation()
                    .minus(m_startView.pairedFieldPose().getRotation())));
            Pose2d error = directPose.relativeTo(m_goal);
            m_lastRight = error.getY();
            m_lastOutwardError = error.getX();
            m_io.reportHopperImageError(measurement.imageHorizontalDegrees()
                - (m_startView == null ? 0.0 : m_startView.imageHorizontalDegrees()));
            m_io.reportHopperCompanionImageError(m_startView == null ? Double.NaN
                : measurement.companionImageHorizontalDegrees() - m_startView.companionImageHorizontalDegrees());

            if (m_controller == null) {
                Translation2d position = directPose.getTranslation();
                double heading = directPose.getRotation().getRadians();
                boolean headingDisagrees = m_frames > 0 && Math.abs(MathUtil.angleModulus(
                    heading - Math.atan2(m_headingSin, m_headingCos))) > Math.toRadians(10.0);
                if (m_clusterFirst == null || m_lastCapture - previousCapture > kMaxVisionAgeSeconds
                        || position.getDistance(m_clusterFirst) > kHopperTagMaxSpreadMeters || headingDisagrees) {
                    clearCluster();
                    m_clusterFirst = position;
                }
                m_sum = m_sum.plus(position);
                m_headingSin += Math.sin(heading);
                m_headingCos += Math.cos(heading);
                if (++m_frames >= kHopperTagAcquireFrames) {
                    Pose2d corrected = new Pose2d(m_sum.div(m_frames), new Rotation2d(m_headingCos, m_headingSin));
                    m_io.resetOdometry(corrected);
                    m_io.onHopperPoseAcquired();
                    m_controller = new HopperAlignmentController(m_goal, kHopperAlignToleranceMeters, m_maxSpeed);
                    m_controller.reset(corrected);
                }
            } else {
                m_io.fuseHopperPairPose(directPose, measurement.timestampSeconds());
                // Both tag observations are required in this pose. Confirm the body pose:
                // aiming either tag to a pixel cannot substitute for translation, and the
                // deliberate bumper stop margin slightly changes the two image angles.
                boolean centered = error.getTranslation().getNorm() <= kHopperAlignToleranceMeters
                    && Math.abs(error.getY()) <= kHopperAlignLateralToleranceMeters
                    && Math.abs(error.getRotation().getRadians()) <= kHopperAlignHeadingToleranceRadians;
                m_confirmations = centered && stopped ? m_confirmations + 1 : 0;
                if (m_confirmations >= kHopperTagConfirmFrames) {
                    finish(true, "hopper position and heading restored using tags " + m_pairName);
                    return;
                }
                if (m_controller.isFinished() && !centered) {
                    m_controller.reset(m_io.getPose());
                }
            }
        }

        if (m_controller == null) {
            m_io.driveRobotRelative(new ChassisSpeeds());
            if (now - m_acquireStarted >= kHopperTagAcquireTimeoutSeconds) {
                finish(false, "alignment failed: no fresh consistent position for hopper tags " + m_pairName);
            }
            return;
        }
        if (now - m_lastCapture > kMaxVisionAgeSeconds) {
            m_io.driveRobotRelative(new ChassisSpeeds());
            report("stopped: waiting for both hopper tags " + m_pairName);
            return;
        }
        ChassisSpeeds requested = m_controller.calculate(m_io.getPose(), speeds);
        report(m_controller.isCreatingClearance() ? "pulling forward before correcting hopper position"
            : "correcting hopper position and heading using tags " + m_pairName);
        // Brake against the measured body position, never against a single tag's image bearing.
        double headingError = m_io.getPose().getRotation().minus(m_goal.getRotation()).getRadians();
        double axialSpeed = speeds.vxMetersPerSecond * Math.cos(headingError);
        double predictedError = m_lastOutwardError + axialSpeed * Math.max(0.0, now - m_lastCapture);
        double stoppingDistance = Math.pow(Math.min(0.0, axialSpeed), 2)
            / (2.0 * kHopperApproachBrakingAccelMetersPerSecondSq);
        if (requested.vxMetersPerSecond < 0.0
                && predictedError <= stoppingDistance + kHopperApproachStopMarginMeters) {
            requested = new ChassisSpeeds();
            report("braking before hopper stand-off");
        }
        m_io.driveRobotRelative(requested);
        if (m_controller.isFinished()) {
            report("confirming hopper position, heading and both tag views");
        }
    }

    private boolean valid(TargetMeasurement measurement, double now) {
        double age = now - measurement.timestampSeconds();
        return measurement.hasHopperPair() && measurement.tagId() == m_tagId
            && measurement.timestampSeconds() > m_captureCutoff && measurement.timestampSeconds() > m_lastCapture
            && Double.isFinite(age) && age <= kMaxVisionAgeSeconds && age >= -kVisionFutureToleranceSeconds
            && Double.isFinite(measurement.fieldHeadingDegrees())
            && Double.isFinite(measurement.imageHorizontalDegrees())
            && Double.isFinite(measurement.companionImageHorizontalDegrees())
            && Double.isFinite(measurement.rightMeters()) && Math.abs(measurement.rightMeters()) <= kHopperTagMaxLateralMeters
            && Double.isFinite(measurement.outwardMeters()) && measurement.outwardMeters() > 0.1
            && measurement.outwardMeters() <= kHopperTagMaxRangeMeters;
    }

    /** Convert Limelight's right/outward coordinates while preserving the supplied gyro heading. */
    public static Pose2d fieldPoseFromTag(TargetMeasurement measurement, Pose2d goal, Pose2d current) {
        return new Pose2d(measurement.toFieldPose(goal).getTranslation(), current.getRotation());
    }

    private void clearCluster() {
        m_sum = Translation2d.kZero;
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
    public boolean isFinished() { return m_finished; }

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
