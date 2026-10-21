package frc.robot.vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.Limelight.PoseMeasurement;

/**
 * Collects a fresh, consistent cluster of AprilTag positions while the drivetrain is stopped.
 * Intended for a deliberate localization stop after emerging from the trench, where encoder
 * drift may exceed the normal per-frame vision correction limit. Never supplies a heading:
 * MegaTag2 uses the gyro heading supplied to the camera and cannot independently recover it.
 */
public final class VisionPoseReacquisition {
    private final int m_requiredFrames;
    private final double m_maxSpreadMeters;
    private final double m_maxCorrectionMeters;
    private final List<Translation2d> m_positions = new ArrayList<>();
    private double m_startedAtSeconds = Double.POSITIVE_INFINITY;
    private double m_lastFrameSeconds = Double.NEGATIVE_INFINITY;

    public VisionPoseReacquisition(int requiredFrames, double maxSpreadMeters, double maxCorrectionMeters) {
        if (requiredFrames < 3 || !Double.isFinite(maxSpreadMeters) || maxSpreadMeters <= 0.0
                || !Double.isFinite(maxCorrectionMeters) || maxCorrectionMeters <= 0.0) {
            throw new IllegalArgumentException("Reacquisition requires at least three frames and positive limits");
        }
        m_requiredFrames = requiredFrames;
        m_maxSpreadMeters = maxSpreadMeters;
        m_maxCorrectionMeters = maxCorrectionMeters;
    }

    /** Start again after stopping, or whenever motion resumes. Old camera captures cannot count. */
    public void reset(double nowSeconds) {
        m_startedAtSeconds = nowSeconds;
        m_lastFrameSeconds = Double.NEGATIVE_INFINITY;
        m_positions.clear();
    }

    /**
     * Adds a raw new camera result, before the normal odometry innovation filter. The caller must
     * stop the drivetrain and reset this collector whenever measured wheel/gyro motion exceeds
     * its settling tolerances. Only multi-tag results may recover drift beyond the normal limit.
     *
     * @return the average X/Y once enough distinct, mutually agreeing captures have arrived
     */
    public Optional<Translation2d> add(PoseMeasurement measurement, Pose2d odometry, double nowSeconds) {
        double frameTime = measurement.timestampSeconds();
        Translation2d position = measurement.pose().getTranslation();
        double age = nowSeconds - frameTime;
        double correction = position.getDistance(odometry.getTranslation());
        if (!Double.isFinite(frameTime) || !Double.isFinite(nowSeconds)
                || !Double.isFinite(position.getX()) || !Double.isFinite(position.getY())
                || !Double.isFinite(measurement.pose().getRotation().getRadians())
                || !Double.isFinite(correction) || !Double.isFinite(measurement.avgTagDistanceMeters())
                || frameTime <= m_startedAtSeconds || frameTime <= m_lastFrameSeconds
                || age > kMaxVisionAgeSeconds || age < -kVisionFutureToleranceSeconds
                || !hasValidTagIds(measurement) || measurement.avgTagDistanceMeters() < 0.0
                || measurement.avgTagDistanceMeters() > kMaxTagDistanceMeters
                || position.getX() < 0.0 || position.getX() > kFieldLengthMeters
                || position.getY() < 0.0 || position.getY() > kFieldWidthMeters
                || correction > m_maxCorrectionMeters
                || (correction > kMaxVisionJumpMeters && measurement.tagCount() < 2)) {
            return Optional.empty();
        }

        // A camera outage or a contradictory position starts a new cluster. Compare every pair,
        // so a chain of small changes cannot slowly walk an averaged solution across the field.
        if (frameTime - m_lastFrameSeconds > kMaxVisionAgeSeconds
                || m_positions.stream().anyMatch(p -> p.getDistance(position) > m_maxSpreadMeters)) {
            m_positions.clear();
        }
        m_lastFrameSeconds = frameTime;
        if (m_positions.size() == m_requiredFrames) {
            m_positions.remove(0);
        }
        m_positions.add(position);
        return getPosition(nowSeconds);
    }

    /** Returns a solution only while its last capture is fresh. */
    public Optional<Translation2d> getPosition(double nowSeconds) {
        if (!Double.isFinite(nowSeconds) || m_positions.size() < m_requiredFrames
                || nowSeconds - m_lastFrameSeconds > kMaxVisionAgeSeconds
                || nowSeconds - m_lastFrameSeconds < -kVisionFutureToleranceSeconds) {
            return Optional.empty();
        }
        Translation2d sum = Translation2d.kZero;
        for (Translation2d position : m_positions) {
            sum = sum.plus(position);
        }
        return Optional.of(sum.div(m_positions.size()));
    }

    public int getFrameCount() {
        return m_positions.size();
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
