package frc.robot.vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.Limelight.PoseMeasurement;

/**
 * Collects fresh AprilTag corrections, comparing vision with odometry at each capture time.
 * This also works while following a path: actual robot movement is removed before comparing
 * frames. Trench mode accepts a validated single trench tag for larger drift. Never changes heading:
 * MegaTag2 uses the gyro heading supplied to the camera and cannot independently recover it.
 */
public final class VisionPoseReacquisition {
    private final int m_requiredFrames;
    private final double m_maxSpreadMeters;
    private final double m_maxCorrectionMeters;
    private final boolean m_trenchOnly;
    private final List<Translation2d> m_offsets = new ArrayList<>();
    private Pose2d m_lastOdometryAtCapture;
    private PoseMeasurement m_lastMeasurement;
    private double m_startedAtSeconds = Double.POSITIVE_INFINITY;
    private double m_lastFrameSeconds = Double.NEGATIVE_INFINITY;

    public VisionPoseReacquisition(int requiredFrames, double maxSpreadMeters, double maxCorrectionMeters) {
        this(requiredFrames, maxSpreadMeters, maxCorrectionMeters, false);
    }

    public static VisionPoseReacquisition forTrenchTags() {
        return new VisionPoseReacquisition(kPostTrenchVisionFrames, kPostTrenchVisionMaxSpreadMeters,
            kPostTrenchVisionMaxCorrectionMeters, true);
    }

    private VisionPoseReacquisition(int requiredFrames, double maxSpreadMeters, double maxCorrectionMeters,
            boolean trenchOnly) {
        if (requiredFrames < 3 || !Double.isFinite(maxSpreadMeters) || maxSpreadMeters <= 0.0
                || !Double.isFinite(maxCorrectionMeters) || maxCorrectionMeters <= 0.0) {
            throw new IllegalArgumentException("Reacquisition requires at least three frames and positive limits");
        }
        m_requiredFrames = requiredFrames;
        m_maxSpreadMeters = maxSpreadMeters;
        m_maxCorrectionMeters = maxCorrectionMeters;
        m_trenchOnly = trenchOnly;
    }

    /** Start a new batch after a correction or a new localization stop; discard older captures. */
    public void reset(double nowSeconds) {
        m_startedAtSeconds = nowSeconds;
        m_lastFrameSeconds = Double.NEGATIVE_INFINITY;
        m_offsets.clear();
        m_lastOdometryAtCapture = null;
        m_lastMeasurement = null;
    }

    /**
     * Adds a raw new camera result, before the normal odometry innovation filter. The caller must
     * supply odometry sampled at the image's capture time, before fusing these corrections.
     * Stationary callers may use the current odometry while holding still. Reset after applying
     * the correction so the next cluster uses a consistent odometry baseline.
     *
     * @return corrected X/Y at the latest capture after enough agreeing offsets have arrived
     */
    public Optional<Translation2d> add(PoseMeasurement measurement, Pose2d odometryAtCapture, double nowSeconds) {
        double frameTime = measurement.timestampSeconds();
        Translation2d position = measurement.pose().getTranslation();
        double age = nowSeconds - frameTime;
        Translation2d offset = position.minus(odometryAtCapture.getTranslation());
        double correction = offset.getNorm();
        if (!Double.isFinite(frameTime) || !Double.isFinite(nowSeconds)
                || !Double.isFinite(position.getX()) || !Double.isFinite(position.getY())
                || !Double.isFinite(measurement.pose().getRotation().getRadians())
                || !Double.isFinite(odometryAtCapture.getRotation().getRadians())
                || !Double.isFinite(correction) || !Double.isFinite(measurement.avgTagDistanceMeters())
                || frameTime <= m_startedAtSeconds || frameTime <= m_lastFrameSeconds
                || age > kMaxVisionAgeSeconds || age < -kVisionFutureToleranceSeconds
                || !hasValidTagIds(measurement) || measurement.avgTagDistanceMeters() < 0.0
                || measurement.avgTagDistanceMeters() > kMaxTagDistanceMeters
                || position.getX() < 0.0 || position.getX() > kFieldLengthMeters
                || position.getY() < 0.0 || position.getY() > kFieldWidthMeters
                || correction > m_maxCorrectionMeters
                || (m_trenchOnly && !isReliableTrenchMeasurement(measurement))
                || (!m_trenchOnly && correction > kMaxVisionJumpMeters && measurement.tagCount() < 2)) {
            return Optional.empty();
        }

        // A camera outage or a contradictory position starts a new cluster. Compare every pair,
        // so a chain of small changes cannot slowly walk an averaged solution across the field.
        if (frameTime - m_lastFrameSeconds > kMaxVisionAgeSeconds
                || m_offsets.stream().anyMatch(p -> p.getDistance(offset) > m_maxSpreadMeters)) {
            m_offsets.clear();
        }
        m_lastFrameSeconds = frameTime;
        if (m_offsets.size() == m_requiredFrames) {
            m_offsets.remove(0);
        }
        m_offsets.add(offset);
        m_lastOdometryAtCapture = odometryAtCapture;
        m_lastMeasurement = measurement;
        return getPosition(nowSeconds);
    }

    /** Returns a solution only while its last capture is fresh. */
    public Optional<Translation2d> getPosition(double nowSeconds) {
        if (!Double.isFinite(nowSeconds) || m_offsets.size() < m_requiredFrames
                || nowSeconds - m_lastFrameSeconds > kMaxVisionAgeSeconds
                || nowSeconds - m_lastFrameSeconds < -kVisionFutureToleranceSeconds) {
            return Optional.empty();
        }
        Translation2d sum = Translation2d.kZero;
        for (Translation2d offset : m_offsets) {
            sum = sum.plus(offset);
        }
        return Optional.of(m_lastOdometryAtCapture.getTranslation().plus(sum.div(m_offsets.size())));
    }

    /** Smoothed X/Y at the latest image timestamp, keeping the historical gyro heading. */
    public Optional<PoseMeasurement> getMeasurement(double nowSeconds) {
        return getPosition(nowSeconds).map(position -> new PoseMeasurement(
            new Pose2d(position, m_lastOdometryAtCapture.getRotation()), m_lastFrameSeconds,
            m_lastMeasurement.tagCount(), m_lastMeasurement.avgTagDistanceMeters(),
            m_lastMeasurement.tagIds(), m_lastMeasurement.tagHorizontalDegrees(), m_lastMeasurement.tagAmbiguities()));
    }

    public static boolean hasTrenchTag(PoseMeasurement measurement) {
        if (measurement.tagIds() == null) {
            return false;
        }
        for (int id : measurement.tagIds()) {
            if (kTrenchAprilTagIds.contains(id)) {
                return true;
            }
        }
        return false;
    }

    private static boolean isReliableTrenchMeasurement(PoseMeasurement measurement) {
        if (measurement.avgTagDistanceMeters() > kTrenchVisionMaxDistanceMeters
                || measurement.tagAmbiguities() == null
                || measurement.tagAmbiguities().length < measurement.tagCount()) {
            return false;
        }
        for (int i = 0; i < measurement.tagCount(); i++) {
            double ambiguity = measurement.tagAmbiguities()[i];
            if (kTrenchAprilTagIds.contains(measurement.tagIds()[i]) && Double.isFinite(ambiguity)
                    && ambiguity >= 0.0 && ambiguity <= kTrenchVisionMaxAmbiguity) {
                return true;
            }
        }
        return false;
    }

    public int getFrameCount() {
        return m_offsets.size();
    }

    private static boolean hasValidTagIds(PoseMeasurement measurement) {
        int[] ids = measurement.tagIds();
        if (measurement.tagCount() < 1 || ids == null || ids.length < measurement.tagCount()) {
            return false;
        }
        for (int i = 0; i < measurement.tagCount(); i++) {
            if (ids[i] <= 0) {
                return false;
            }
            for (int j = 0; j < i; j++) {
                if (ids[i] == ids[j]) {
                    return false;
                }
            }
        }
        return true;
    }
}
