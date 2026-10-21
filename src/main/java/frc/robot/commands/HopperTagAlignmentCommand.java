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

/** Lines up with the hopper using image centering or a separately requested lateral return offset. */
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
        default void reportHopperImageError(double degrees) {}
        default void reportHopperLeftTarget(double meters) {}
        default void onHopperPoseAcquired() {}
    }

    private final IO m_io;
    private final Pose2d m_goal;
    private final int m_tagId;
    private final DoubleSupplier m_clock;
    private final BooleanSupplier m_visionEnabled;
    private final double m_maxSpeed;
    private final double m_cameraRightTarget;
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
    private double m_acquireStarted;
    private double m_captureCutoff;
    private double m_lastCapture;
    private double m_lastRight;
    private double m_lastOutward;
    private double m_lastImageAngle;
    private double m_lastImageCapture;
    private double m_lastImageHeading;
    private boolean m_finished;
    private boolean m_success;
    private String m_status;

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled) {
        this(io, requirement, goal, tagId, clock, visionEnabled, kHopperAlignMaxSpeedMetersPerSecond);
    }

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled, double maxSpeed) {
        this(io, requirement, goal, tagId, clock, visionEnabled, maxSpeed, 0.0);
    }

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled, double maxSpeed, double leftOffsetMeters) {
        m_io = io;
        m_goal = goal;
        m_tagId = tagId;
        m_clock = clock;
        m_visionEnabled = visionEnabled;
        m_maxSpeed = maxSpeed;
        m_cameraRightTarget = kHopperCameraRightTargetMeters - leftOffsetMeters;
        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        m_started = m_clock.getAsDouble();
        m_acquireStarted = m_started;
        m_captureCutoff = m_started;
        m_lastCapture = Double.NEGATIVE_INFINITY;
        m_lastRight = Double.NaN;
        m_lastOutward = Double.NaN;
        m_lastImageAngle = Double.NaN;
        m_lastImageCapture = Double.NEGATIVE_INFINITY;
        m_controller = null;
        m_cameraCenteredGoal = m_goal;
        clearCluster();
        m_confirmations = 0;
        m_finished = false;
        m_success = false;
        m_io.beginHopperTagAlignment(m_tagId);
        m_io.reportHopperLeftTarget(-m_cameraRightTarget);
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
        if (now - m_started >= (isOffsetReturn() ? kHopperReturnAlignTimeoutSeconds : kHopperAlignTimeoutSeconds)) {
            finish(false, isOffsetReturn() ? "alignment timed out: left return target not confirmed"
                : "alignment timed out: tag " + m_tagId + " center not confirmed");
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
                m_lastRight = measurement.cameraRightMeters() - m_cameraRightTarget;
                m_lastOutward = measurement.outwardMeters();
                m_io.reportHopperImageError(measurement.imageHorizontalDegrees());
                report(m_status);
                if (!Double.isFinite(measurement.fieldHeadingDegrees())) {
                    // PathPlanner seeds field heading. Keep using the gyro for every capture
                    // when MegaTag1 is absent, including while removing yaw from image error.
                    measurement = new TargetMeasurement(measurement.tagId(), measurement.rightMeters(),
                        measurement.outwardMeters(), measurement.timestampSeconds(),
                        m_io.getPose().getRotation().getDegrees(), measurement.cameraRightMeters(),
                        measurement.cameraOutwardMeters(), measurement.imageHorizontalDegrees());
                }
                Pose2d directPose = fieldPoseFromTag(measurement, m_goal, m_io.getPose());
                if (Double.isFinite(measurement.imageHorizontalDegrees())) {
                    m_lastImageAngle = measurement.imageHorizontalDegrees();
                    m_lastImageCapture = measurement.timestampSeconds();
                    m_lastImageHeading = Math.toRadians(measurement.fieldHeadingDegrees());
                }
                if (m_controller == null) {
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
                    m_goalSum = m_goalSum.plus(cameraCenteredGoal(measurement, m_goal, m_cameraRightTarget).getTranslation());
                    m_headingSin += Math.sin(headingRadians);
                    m_headingCos += Math.cos(headingRadians);
                    m_frames++;
                    if (m_frames >= kHopperTagAcquireFrames) {
                        Pose2d corrected = new Pose2d(m_sum.div(m_frames),
                            new Rotation2d(m_headingCos, m_headingSin));
                        m_io.resetOdometry(corrected);
                        m_io.onHopperPoseAcquired();
                        m_cameraCenteredGoal = new Pose2d(m_goalSum.div(m_frames), m_goal.getRotation());
                        m_controller = new HopperAlignmentController(m_cameraCenteredGoal,
                            kHopperAlignToleranceMeters, m_maxSpeed);
                        m_controller.reset(corrected);
                        report("moving to hopper return target using tag " + m_tagId);
                    }
                } else {
                    // Fuse at capture time to account for movement during camera latency.
                    m_io.fuseHopperTagPose(directPose, measurement.timestampSeconds());
                    if (Double.isFinite(measurement.imageHorizontalDegrees())) {
                        m_cameraCenteredGoal = cameraCenteredGoal(measurement, m_goal, m_cameraRightTarget);
                        m_controller.updateTarget(m_cameraCenteredGoal, m_io.getPose());
                    }
                    // A large intentional lateral offset can put tag 26 outside the image.
                    // Other mapped tags can still confirm the position from a fresh field pose.
                    // The zero-offset mode continues to require tag 26's actual image center.
                    boolean lateralTargetReached = isOffsetReturn()
                        ? directPose.getTranslation().getDistance(m_cameraCenteredGoal.getTranslation())
                            <= kHopperAlignToleranceMeters
                        : Double.isFinite(measurement.imageHorizontalDegrees())
                            && Math.abs(measurement.imageHorizontalDegrees()) <= kHopperImageCenterToleranceDegrees;
                    boolean centered = lateralTargetReached
                        && Math.abs(measurement.outwardMeters() - kHopperTagStandOffMeters) <= kHopperAlignToleranceMeters
                        && Math.abs(m_goal.getRotation().minus(m_io.getPose().getRotation()).getRadians())
                            <= kHopperAlignHeadingToleranceRadians;
                    m_confirmations = centered && stopped ? m_confirmations + 1 : 0;
                    if (m_confirmations >= kHopperTagConfirmFrames) {
                        finish(true, isOffsetReturn() ? "aligned at " + (-m_cameraRightTarget)
                            + " m left return target using fresh vision"
                            : "tag " + m_tagId + " centered in Limelight image");
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
            m_io.driveRobotRelative(new ChassisSpeeds());
            if (now - m_acquireStarted >= kHopperTagAcquireTimeoutSeconds) {
                finish(false, "alignment failed: no fresh consistent position for hopper tag " + m_tagId);
            }
        } else {
            if (now - m_lastCapture > kMaxVisionAgeSeconds) {
                m_io.driveRobotRelative(new ChassisSpeeds());
                report("stopped: waiting for fresh hopper position");
                return;
            }
            ChassisSpeeds requested = m_controller.calculate(m_io.getPose(), speeds);
            report(m_controller.isCreatingClearance() ? "pulling forward to recenter tag " + m_tagId
                : isOffsetReturn() ? "correcting to " + (-m_cameraRightTarget) + " m left return target"
                : "correcting position for tag " + m_tagId + " image center");
            if (!isOffsetReturn() && m_controller.isFinished() && now - m_lastImageCapture <= kMaxVisionAgeSeconds) {
                // Once physically lined up, remove the remaining small pixel error. Predict
                // its change since the capture from the gyro, rather than chasing stale tx.
                double imageError = MathUtil.angleModulus(Math.toRadians(m_lastImageAngle)
                    + m_io.getPose().getRotation().getRadians() - m_lastImageHeading);
                double aimedHeadingError = MathUtil.angleModulus(m_io.getPose().getRotation().getRadians()
                    - imageError - m_goal.getRotation().getRadians());
                if (Math.abs(aimedHeadingError) <= kHopperAlignHeadingToleranceRadians) {
                    requested = new ChassisSpeeds(0.0, 0.0, MathUtil.clamp(-kHopperAlignHeadingP * imageError,
                        -kHopperImageFineTurnMaxRadPerSec, kHopperImageFineTurnMaxRadPerSec));
                }
            }
            double headingError = m_io.getPose().getRotation().minus(m_goal.getRotation()).getRadians();
            double axialSpeed = speeds.vxMetersPerSecond * Math.cos(headingError);
            double predictedRange = m_lastOutward + axialSpeed * Math.max(0.0, now - m_lastCapture);
            double stoppingDistance = Math.pow(Math.min(0.0, axialSpeed), 2)
                / (2.0 * kHopperApproachBrakingAccelMetersPerSecondSq);
            if (requested.vxMetersPerSecond < 0.0
                    && predictedRange - kHopperTagStandOffMeters <= stoppingDistance + kHopperApproachStopMarginMeters) {
                requested = new ChassisSpeeds();
                report("braking before hopper stand-off");
            }
            m_io.driveRobotRelative(requested);
            if (m_controller.isFinished()) {
                report(isOffsetReturn() ? "confirming left return target with fresh vision"
                    : now - m_lastImageCapture <= kMaxVisionAgeSeconds
                    ? "confirming tag " + m_tagId + " image center"
                    : "position acquired; waiting to see tag " + m_tagId + " in image");
            }
        }
    }

    /** Convert Limelight's right/outward tag coordinates to WPILib blue-origin field position. */
    public static Pose2d fieldPoseFromTag(TargetMeasurement measurement, Pose2d goal, Pose2d current) {
        // The goal includes the return bias; the measured tag position does not. Remove that
        // bias from the reference so it changes physical travel instead of shifting odometry.
        Translation2d relative = new Translation2d(
            measurement.outwardMeters() - kHopperTagStandOffMeters,
            measurement.rightMeters() - kHopperPathRightReferenceMeters);
        return new Pose2d(goal.getTranslation().plus(relative.rotateBy(goal.getRotation())), current.getRotation());
    }

    /** Shift the robot goal for the measured camera mount offset while preserving bumper clearance. */
    public static Pose2d cameraCenteredGoal(TargetMeasurement measurement, Pose2d nominalGoal) {
        return cameraCenteredGoal(measurement, nominalGoal, kHopperCameraRightTargetMeters);
    }

    private boolean isOffsetReturn() {
        return Math.abs(m_cameraRightTarget - kHopperCameraRightTargetMeters) > 1e-9;
    }

    private static Pose2d cameraCenteredGoal(TargetMeasurement measurement, Pose2d nominalGoal,
            double cameraRightTarget) {
        double fieldHeading = Double.isFinite(measurement.fieldHeadingDegrees())
            ? measurement.fieldHeadingDegrees() : nominalGoal.getRotation().getDegrees();
        double headingRelative = MathUtil.angleModulus(Math.toRadians(fieldHeading)
            - nominalGoal.getRotation().getRadians());
        // A centered field estimate is insufficient if the camera still sees the tag off to
        // one side. Convert the observed image bearing to lateral error, removing robot yaw
        // so that turning alone cannot masquerade as correcting the robot's position.
        double cameraRight = measurement.cameraRightMeters();
        if (Double.isFinite(measurement.imageHorizontalDegrees())) {
            cameraRight = measurement.cameraOutwardMeters() * Math.tan(MathUtil.angleModulus(
                headingRelative - Math.toRadians(measurement.imageHorizontalDegrees())));
        }
        Translation2d robotToCamera = new Translation2d(
            measurement.cameraOutwardMeters() - measurement.outwardMeters(),
            cameraRight - measurement.rightMeters());
        // Rotate the measured mount offset to where it will be at the final robot heading.
        Translation2d offsetAtGoal = robotToCamera.rotateBy(nominalGoal.getRotation()
            .minus(Rotation2d.fromDegrees(fieldHeading)));
        Translation2d shift = new Translation2d(0.0,
            cameraRightTarget - kHopperPathRightReferenceMeters - offsetAtGoal.getY())
            .rotateBy(nominalGoal.getRotation());
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
