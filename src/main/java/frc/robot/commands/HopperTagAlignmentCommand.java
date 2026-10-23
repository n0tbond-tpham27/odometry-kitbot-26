package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import frc.robot.vision.HopperViewAlignment;

/** Returns the robot to its startup position and heading using both hopper tags. */
public final class HopperTagAlignmentCommand extends Command {
    public interface IO {
        Pose2d getPose();
        default Rotation2d getHeadingAt(double timestampSeconds) { return getPose().getRotation(); }
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
        default void reportHopperViewTarget(HopperViewAlignment.View target, String source) {}
        default void reportHopperViewError(double mainY, double companionY, double headingDegrees) {}
        default void onHopperPoseAcquired() {}
    }

    private final IO m_io;
    private final Pose2d m_goal;
    private final Pose2d m_referencePose;
    private final double m_leftTrimMeters;
    private final int m_tagId;
    private final DoubleSupplier m_clock;
    private final BooleanSupplier m_visionEnabled;
    private final double m_maxSpeed;
    private final Reference m_startView;
    private final String m_pairName;
    private final HopperViewAlignment.View m_viewTarget;
    private final boolean m_matchView;
    private boolean m_usingViewPose;
    private HopperAlignmentController m_controller;
    private Translation2d m_sum;
    private Translation2d m_clusterFirst;
    private int m_frames;
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
        this(io, requirement, goal, tagId, clock, visionEnabled, maxSpeed, startView, 0.0);
    }

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled, double maxSpeed, Reference startView,
            double leftTrimMeters) {
        this(io, requirement, goal, tagId, clock, visionEnabled, maxSpeed, startView, leftTrimMeters, false);
    }

    public HopperTagAlignmentCommand(IO io, Subsystem requirement, Pose2d goal, int tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled, double maxSpeed, Reference startView,
            double leftTrimMeters, boolean usePhotoFallback) {
        m_io = io;
        m_startView = startView != null && startView.tagId() == tagId ? startView : null;
        m_referencePose = m_startView != null ? m_startView.startingPose() : goal;
        m_goal = withDriverStationLeftTrim(m_referencePose, leftTrimMeters);
        m_leftTrimMeters = leftTrimMeters;
        m_matchView = Math.abs(leftTrimMeters) < 1e-9 && (m_startView != null || usePhotoFallback);
        m_viewTarget = m_startView != null ? HopperViewAlignment.from(m_startView) : HopperViewAlignment.photo();
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
        m_usingViewPose = false;
        clearCluster();
        m_confirmations = 0;
        m_finished = false;
        m_success = false;
        m_io.beginHopperTagAlignment(m_tagId);
        if (m_startView != null) {
            m_io.reportHopperStartViewTarget(m_startView);
        }
        if (m_startView == null || Math.abs(m_leftTrimMeters) > 1e-9) {
            m_io.reportHopperLeftTarget(m_leftTrimMeters);
        }
        if (m_matchView) {
            m_io.reportHopperViewTarget(m_viewTarget, m_startView != null ? "live capture matching hub photo"
                : "photo estimate using LL4 factory lens");
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
        if (now - m_started >= (m_startView != null || m_matchView ? kHopperReturnAlignTimeoutSeconds : kHopperAlignTimeoutSeconds)) {
            finish(false, "alignment timed out: hopper position and heading not restored using tags " + m_pairName);
            return;
        }
        if (now - m_lastCapture > kMaxVisionAgeSeconds) {
            m_confirmations = 0;
        }
        ChassisSpeeds speeds = m_io.getRobotRelativeSpeeds();
        boolean stopped = Math.abs(speeds.vxMetersPerSecond) <= kDriveStraightSpeedToleranceMetersPerSecond
            && Math.abs(speeds.omegaRadiansPerSecond) <= kHopperAlignTurnSpeedToleranceRadPerSec;
        if (m_controller == null && Math.abs(m_io.getPose().getRotation().minus(m_goal.getRotation()).getRadians())
                > Math.toRadians(75.0)) {
            finish(false, "alignment failed: gyro field heading needs the known hopper start reference");
            return;
        }
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
            // A return trim moves the physical goal, never the measured coordinate frame.
            Pose2d rawPose = measurement.toFieldPose(m_referencePose);
            Rotation2d gyroHeading = m_io.getHeadingAt(measurement.timestampSeconds());
            // Compare field X/Y with the startup reference, but retain the gyro heading.
            // The camera may correct translation without changing the gyro's field reference.
            Pose2d directPose = m_startView == null ? new Pose2d(rawPose.getTranslation(), gyroHeading) : new Pose2d(
                m_startView.startingPose().getTranslation().plus(rawPose.getTranslation()
                    .minus(m_startView.pairedFieldPose().getTranslation())),
                gyroHeading);
            boolean previouslyUsingView = m_usingViewPose;
            if (m_matchView && (m_usingViewPose
                    || directPose.getTranslation().getDistance(m_goal.getTranslation()) < 0.25)) {
                Optional<Pose2d> visual = HopperViewAlignment.estimatePose(measurement, m_viewTarget, m_goal, gyroHeading);
                if (visual.isPresent()) {
                    directPose = visual.get();
                    m_usingViewPose = true;
                } else {
                    m_usingViewPose = false;
                }
            }
            if (m_controller != null && previouslyUsingView != m_usingViewPose) {
                m_controller = newController();
                m_controller.reset(directPose);
            }
            Pose2d error = directPose.relativeTo(m_goal);
            m_lastRight = error.getY();
            m_lastOutwardError = error.getX();
            m_io.reportHopperImageError(measurement.imageHorizontalDegrees()
                - (m_startView == null ? 0.0 : m_startView.imageHorizontalDegrees()));
            m_io.reportHopperCompanionImageError(m_startView == null ? Double.NaN
                : measurement.companionImageHorizontalDegrees() - m_startView.companionImageHorizontalDegrees());
            if (m_matchView) {
                m_io.reportHopperImageError(measurement.imageHorizontalDegrees() - m_viewTarget.mainX());
                m_io.reportHopperCompanionImageError(measurement.companionImageHorizontalDegrees() - m_viewTarget.companionX());
                m_io.reportHopperViewError(measurement.imageVerticalDegrees() - m_viewTarget.mainY(),
                    measurement.companionImageVerticalDegrees() - m_viewTarget.companionY(),
                    gyroHeading.minus(m_goal.getRotation()).getDegrees());
            }

            if (m_controller == null) {
                Translation2d position = directPose.getTranslation();
                if (m_clusterFirst == null || m_lastCapture - previousCapture > kMaxVisionAgeSeconds
                        || position.getDistance(m_clusterFirst) > kHopperTagMaxSpreadMeters) {
                    clearCluster();
                    m_clusterFirst = position;
                }
                m_sum = m_sum.plus(position);
                if (++m_frames >= kHopperTagAcquireFrames) {
                    Pose2d corrected = new Pose2d(m_sum.div(m_frames), m_io.getPose().getRotation());
                    m_io.resetOdometry(corrected);
                    m_io.onHopperPoseAcquired();
                    m_controller = newController();
                    m_controller.reset(corrected);
                }
            } else {
                m_io.fuseHopperTagPose(directPose, measurement.timestampSeconds());
                // Require the chassis pose, gyro heading, and both tag views to agree.
                // A turn that merely puts one tag at the crosshair is insufficient.
                boolean centered = error.getTranslation().getNorm() <= kHopperAlignToleranceMeters
                    && Math.abs(error.getY()) <= kHopperAlignLateralToleranceMeters
                    && Math.abs(error.getRotation().getRadians()) <= kHopperAlignHeadingToleranceRadians
                    && (!m_matchView || (m_usingViewPose && m_viewTarget.matches(measurement, kHopperImageCenterToleranceDegrees)));
                m_confirmations = centered && stopped ? m_confirmations + 1 : 0;
                if (m_confirmations >= kHopperTagConfirmFrames) {
                    finish(true, "hopper return target reached using tags " + m_pairName);
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
        double stopMargin = m_usingViewPose ? kHopperViewFineStopMarginMeters : kHopperApproachStopMarginMeters;
        if (requested.vxMetersPerSecond < 0.0 && predictedError <= stoppingDistance + stopMargin) {
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
            && Double.isFinite(measurement.imageHorizontalDegrees())
            && Double.isFinite(measurement.companionImageHorizontalDegrees())
            && Double.isFinite(measurement.rightMeters()) && Math.abs(measurement.rightMeters()) <= kHopperTagMaxLateralMeters
            && Double.isFinite(measurement.outwardMeters()) && measurement.outwardMeters() > 0.1
            && measurement.outwardMeters() <= kHopperTagMaxRangeMeters;
    }

    private HopperAlignmentController newController() {
        return new HopperAlignmentController(m_goal,
            m_usingViewPose ? kHopperViewFinePositionToleranceMeters : kHopperAlignToleranceMeters, m_maxSpeed,
            m_usingViewPose ? kHopperViewFineStopMarginMeters : kHopperApproachStopMarginMeters,
            m_usingViewPose ? 4.0 : 2.5);
    }

    /** Move the return goal toward the driver's left, mirrored with the alliance heading. */
    public static Pose2d withDriverStationLeftTrim(Pose2d reference, double leftMeters) {
        Translation2d shift = new Translation2d(0.0, -leftMeters).rotateBy(reference.getRotation());
        return new Pose2d(reference.getTranslation().plus(shift), reference.getRotation());
    }

    /** Convert Limelight's right/outward coordinates while preserving the supplied gyro heading. */
    public static Pose2d fieldPoseFromTag(TargetMeasurement measurement, Pose2d goal, Pose2d current) {
        return new Pose2d(measurement.toFieldPose(goal).getTranslation(), current.getRotation());
    }

    private void clearCluster() {
        m_sum = Translation2d.kZero;
        m_clusterFirst = null;
        m_frames = 0;
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
