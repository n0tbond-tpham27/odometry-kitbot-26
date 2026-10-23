package frc.robot.vision;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.vision.Limelight.TargetMeasurement;

/** Records one stationary startup view, then freezes it until the next autonomous start. */
public final class HopperStartView {
    public record Reference(int tagId, Pose2d startingPose, double robotOutwardMeters,
            double cameraOutwardMeters, double imageHorizontalDegrees,
            double companionImageHorizontalDegrees, Pose2d pairedFieldPose,
            double imageVerticalDegrees, double companionImageVerticalDegrees) {}

    private final boolean m_requirePhotoMatch;

    public HopperStartView() { this(false); }

    public HopperStartView(boolean requirePhotoMatch) { m_requirePhotoMatch = requirePhotoMatch; }

    private int m_tagId;
    private Pose2d m_startingPose;
    private double m_cutoff;
    private double m_lastCapture;
    private TargetMeasurement m_first;
    private int m_frames;
    private double m_robotRangeSum;
    private double m_cameraRangeSum;
    private double m_angleSum;
    private double m_companionAngleSum;
    private double m_verticalSum;
    private double m_companionVerticalSum;
    private Translation2d m_positionSum;
    private double m_headingSin;
    private double m_headingCos;
    private Reference m_reference;

    public void reset(int tagId, Pose2d startingPose, double now) {
        m_tagId = tagId;
        m_startingPose = startingPose;
        m_cutoff = now;
        m_lastCapture = Double.NEGATIVE_INFINITY;
        m_reference = null;
        clearCluster();
    }

    public Optional<Reference> getReference() {
        return Optional.ofNullable(m_reference);
    }

    public void observeMotion(ChassisSpeeds speeds, double now) {
        if (m_reference == null && !isStopped(speeds)) {
            m_cutoff = now;
            clearCluster();
        }
    }

    public void add(TargetMeasurement sample, ChassisSpeeds speeds, double now) {
        if (m_reference != null) {
            return;
        }
        observeMotion(speeds, now);
        if (!isStopped(speeds)) {
            return;
        }
        double age = now - sample.timestampSeconds();
        if (!sample.hasHopperPair() || !Double.isFinite(sample.fieldHeadingDegrees())
                || !Double.isFinite(sample.rightMeters())
                || !Double.isFinite(sample.companionImageHorizontalDegrees())
                || Math.abs(sample.companionImageHorizontalDegrees()) >= 70.0
                || sample.tagId() != m_tagId || sample.timestampSeconds() <= m_cutoff
                || sample.timestampSeconds() <= m_lastCapture || !Double.isFinite(age)
                || age > kMaxVisionAgeSeconds || age < -kVisionFutureToleranceSeconds
                || !Double.isFinite(sample.imageHorizontalDegrees()) || Math.abs(sample.imageHorizontalDegrees()) >= 70.0
                || !validRange(sample.outwardMeters()) || !validRange(sample.cameraOutwardMeters())) {
            return;
        }
        if (m_requirePhotoMatch && !HopperViewAlignment.photo().matches(sample, kHubPhotoCaptureToleranceDegrees)) {
            clearCluster();
            return;
        }
        if (m_first == null || sample.timestampSeconds() - m_lastCapture > kMaxVisionAgeSeconds
                || Math.abs(sample.outwardMeters() - m_first.outwardMeters()) > kHopperStartViewMaxRangeSpreadMeters
                || Math.abs(sample.cameraOutwardMeters() - m_first.cameraOutwardMeters()) > kHopperStartViewMaxRangeSpreadMeters
                || Math.abs(sample.imageHorizontalDegrees() - m_first.imageHorizontalDegrees()) > kHopperStartViewMaxAngleSpreadDegrees
                || Math.abs(sample.companionImageHorizontalDegrees() - m_first.companionImageHorizontalDegrees()) > kHopperStartViewMaxAngleSpreadDegrees
                || (Double.isFinite(sample.imageVerticalDegrees()) && Double.isFinite(m_first.imageVerticalDegrees())
                    && Math.abs(sample.imageVerticalDegrees() - m_first.imageVerticalDegrees()) > kHopperStartViewMaxAngleSpreadDegrees)
                || (Double.isFinite(sample.companionImageVerticalDegrees()) && Double.isFinite(m_first.companionImageVerticalDegrees())
                    && Math.abs(sample.companionImageVerticalDegrees() - m_first.companionImageVerticalDegrees()) > kHopperStartViewMaxAngleSpreadDegrees)
                || Math.abs(MathUtil.angleModulus(Math.toRadians(sample.fieldHeadingDegrees()
                    - m_first.fieldHeadingDegrees()))) > Math.toRadians(kHopperStartViewMaxAngleSpreadDegrees)
                || sample.toFieldPose(m_startingPose).getTranslation().getDistance(
                    m_first.toFieldPose(m_startingPose).getTranslation()) > kHopperStartViewMaxRangeSpreadMeters) {
            clearCluster();
            m_first = sample;
        }
        m_lastCapture = sample.timestampSeconds();
        m_robotRangeSum += sample.outwardMeters();
        m_cameraRangeSum += sample.cameraOutwardMeters();
        m_angleSum += sample.imageHorizontalDegrees();
        m_companionAngleSum += sample.companionImageHorizontalDegrees();
        m_verticalSum += sample.imageVerticalDegrees();
        m_companionVerticalSum += sample.companionImageVerticalDegrees();
        m_positionSum = m_positionSum.plus(sample.toFieldPose(m_startingPose).getTranslation());
        m_headingSin += Math.sin(Math.toRadians(sample.fieldHeadingDegrees()));
        m_headingCos += Math.cos(Math.toRadians(sample.fieldHeadingDegrees()));
        if (++m_frames >= kHopperTagAcquireFrames) {
            m_reference = new Reference(m_tagId, m_startingPose, m_robotRangeSum / m_frames,
                m_cameraRangeSum / m_frames, m_angleSum / m_frames, m_companionAngleSum / m_frames,
                new Pose2d(m_positionSum.div(m_frames), new Rotation2d(m_headingCos, m_headingSin)),
                m_verticalSum / m_frames, m_companionVerticalSum / m_frames);
        }
    }

    private static boolean validRange(double meters) {
        return Double.isFinite(meters) && meters > 0.1 && meters <= kHopperTagMaxRangeMeters;
    }

    private static boolean isStopped(ChassisSpeeds speeds) {
        return Double.isFinite(speeds.vxMetersPerSecond) && Double.isFinite(speeds.omegaRadiansPerSecond)
            && Math.abs(speeds.vxMetersPerSecond) <= kDriveStraightSpeedToleranceMetersPerSecond
            && Math.abs(speeds.omegaRadiansPerSecond) <= kHopperAlignTurnSpeedToleranceRadPerSec;
    }

    private void clearCluster() {
        m_first = null;
        m_frames = 0;
        m_robotRangeSum = 0.0;
        m_cameraRangeSum = 0.0;
        m_angleSum = 0.0;
        m_companionAngleSum = 0.0;
        m_verticalSum = 0.0;
        m_companionVerticalSum = 0.0;
        m_positionSum = Translation2d.kZero;
        m_headingSin = 0.0;
        m_headingCos = 0.0;
    }
}
