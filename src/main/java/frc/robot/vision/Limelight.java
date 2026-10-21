package frc.robot.vision;

import java.util.Optional;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;

/**
 * Reads the robot's field position from a Limelight's AprilTag tracking (MegaTag2) over
 * NetworkTables. MegaTag2 combines the tags it sees with the heading we send it every loop
 * (see {@link #setRobotOrientation}), which makes it accurate even with only one or two tags.
 *
 * <p>The Limelight must have the field map with the tags' positions (the 2026 field map is the
 * default) and its camera position on the robot set in its web interface.
 */
public class Limelight {
    /** A robot position measured from AprilTags, ready for addVisionMeasurement(). */
    public record PoseMeasurement(
        Pose2d pose, double timestampSeconds, int tagCount, double avgTagDistanceMeters, int[] tagIds) {}

    /** Robot and camera positions relative to the selected tag's center, independent of odometry. */
    public record TargetMeasurement(int tagId, double rightMeters, double outwardMeters, double timestampSeconds,
            double fieldHeadingDegrees, double cameraRightMeters, double cameraOutwardMeters) {
        public TargetMeasurement(int tagId, double rightMeters, double outwardMeters, double timestampSeconds,
                double fieldHeadingDegrees) {
            this(tagId, rightMeters, outwardMeters, timestampSeconds, fieldHeadingDegrees, rightMeters, outwardMeters);
        }

        public TargetMeasurement(int tagId, double rightMeters, double outwardMeters, double timestampSeconds) {
            this(tagId, rightMeters, outwardMeters, timestampSeconds, Double.NaN);
        }
    }

    // Layout of the botpose arrays: x, y, z, roll, pitch, yaw, latency (ms), tag count,
    // tag span, average tag distance, average tag area, then 7 values per tag (ID first).
    private static final int kLatencyIndex = 6;
    private static final int kTagCountIndex = 7;
    private static final int kAvgDistanceIndex = 9;
    private static final int kMinLength = 11;
    private static final int kValuesPerTag = 7;

    private final DoubleArraySubscriber m_botPoseSub;
    private final DoubleArrayPublisher m_orientationPub;
    private final DoubleArraySubscriber m_targetPoseSub;
    private final DoubleArraySubscriber m_cameraTargetPoseSub;
    private final DoubleArraySubscriber m_targetStatsSub;
    private final DoubleArraySubscriber m_tagFieldPoseSub;
    private final edu.wpi.first.networktables.DoublePublisher m_priorityPub;
    private long m_lastTargetTimestampMicros = 0;
    /** NetworkTables timestamp of the last result returned, so each frame is used only once. */
    private long m_lastTimestampMicros = 0;

    public Limelight(String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        // Robot position in blue-origin field coordinates, i.e. the same as our odometry
        // keepDuplicates: a new frame that happens to give the exact same numbers still counts
        m_botPoseSub = table.getDoubleArrayTopic("botpose_orb_wpiblue")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
        m_orientationPub = table.getDoubleArrayTopic("robot_orientation_set").publish();
        m_targetPoseSub = table.getDoubleArrayTopic("botpose_targetspace")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
        m_cameraTargetPoseSub = table.getDoubleArrayTopic("camerapose_targetspace")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
        // t2d gives validity, tag ID, and both latencies from the same camera frame.
        m_targetStatsSub = table.getDoubleArrayTopic("t2d")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
        // MegaTag1 supplies an independent heading for initial gyro alignment; MegaTag2 cannot.
        m_tagFieldPoseSub = table.getDoubleArrayTopic("botpose_wpiblue")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
        m_priorityPub = table.getDoubleTopic("priorityid").publish();
    }

    /** Select the hopper tag for single-target measurements without filtering field localization. */
    public void setPriorityTagId(int tagId) {
        m_priorityPub.set(tagId);
    }

    public Optional<TargetMeasurement> getNewTargetPose(int requiredTagId) {
        TimestampedDoubleArray pose = m_targetPoseSub.getAtomic();
        TimestampedDoubleArray stats = m_targetStatsSub.getAtomic();
        if (pose.timestamp == m_lastTargetTimestampMicros) {
            return Optional.empty();
        }
        Optional<TargetMeasurement> result = parseTargetPose(
            pose.value, pose.timestamp, stats.value, stats.timestamp, requiredTagId);
        // Retry a pose whose matching stats have not arrived yet.
        if (result.isPresent()) {
            m_lastTargetTimestampMicros = pose.timestamp;
            TimestampedDoubleArray cameraPose = m_cameraTargetPoseSub.getAtomic();
            TargetMeasurement withCamera = withCameraPose(
                result.get(), pose.timestamp, cameraPose.value, cameraPose.timestamp);
            TimestampedDoubleArray fieldPose = m_tagFieldPoseSub.getAtomic();
            return Optional.of(withFieldHeading(withCamera, pose.timestamp, fieldPose.value, fieldPose.timestamp));
        }
        return result;
    }

    static TargetMeasurement withCameraPose(TargetMeasurement measurement, long poseTimestampMicros,
            double[] cameraPose, long cameraTimestampMicros) {
        boolean valid = cameraPose.length >= 6 && cameraTimestampMicros > 0
            && Math.abs(poseTimestampMicros - cameraTimestampMicros) <= 20000
            && Double.isFinite(cameraPose[0]) && Double.isFinite(cameraPose[2])
            && cameraPose[2] > 0.0 && cameraPose[2] <= kHopperTagMaxRangeMeters
            && Math.abs(cameraPose[0]) <= kHopperTagMaxLateralMeters;
        return new TargetMeasurement(measurement.tagId(), measurement.rightMeters(), measurement.outwardMeters(),
            measurement.timestampSeconds(), measurement.fieldHeadingDegrees(),
            valid ? cameraPose[0] : Double.NaN, valid ? cameraPose[2] : Double.NaN);
    }

    static TargetMeasurement withFieldHeading(TargetMeasurement measurement, long poseTimestampMicros,
            double[] fieldPose, long fieldTimestampMicros) {
        boolean valid = fieldPose.length >= kMinLength + kValuesPerTag
            && Math.abs(poseTimestampMicros - fieldTimestampMicros) <= 20000
            && Double.isFinite(fieldPose[5]) && Double.isFinite(fieldPose[kTagCountIndex])
            && fieldPose[kTagCountIndex] >= 1.0;
        if (valid && fieldPose[kTagCountIndex] == 1.0) {
            double ambiguity = fieldPose[kMinLength + 6];
            valid = Double.isFinite(ambiguity) && ambiguity >= 0.0 && ambiguity <= 0.4;
        }
        return new TargetMeasurement(measurement.tagId(), measurement.rightMeters(), measurement.outwardMeters(),
            measurement.timestampSeconds(), valid ? fieldPose[5] : Double.NaN,
            measurement.cameraRightMeters(), measurement.cameraOutwardMeters());
    }

    static Optional<TargetMeasurement> parseTargetPose(
            double[] pose, long poseTimestampMicros, double[] stats, long statsTimestampMicros, int requiredTagId) {
        if (pose.length < 6 || stats.length < 10 || poseTimestampMicros <= 0 || statsTimestampMicros <= 0
                || Math.abs(poseTimestampMicros - statsTimestampMicros) > 20000
                || stats[0] != 1.0 || stats[9] != requiredTagId
                || !Double.isFinite(pose[0]) || !Double.isFinite(pose[2])
                || !Double.isFinite(stats[2]) || !Double.isFinite(stats[3])
                || stats[2] < 0.0 || stats[3] < 0.0
                || pose[2] <= 0.1 || pose[2] > kHopperTagMaxRangeMeters
                || Math.abs(pose[0]) > kHopperTagMaxLateralMeters) {
            return Optional.empty();
        }
        // Limelight target axes: +X right when looking at the tag, +Z out of its face.
        double captureSeconds = poseTimestampMicros / 1e6 - (stats[2] + stats[3]) / 1000.0;
        return Optional.of(new TargetMeasurement(requiredTagId, pose[0], pose[2], captureSeconds));
    }

    /**
     * Sends the robot's field heading to the Limelight; MegaTag2 needs this every loop.
     *
     * @param headingDegrees field heading, CCW-positive, 0 = facing away from the blue wall
     * @param yawRateDegreesPerSecond how fast the robot is turning, CCW-positive
     */
    public void setRobotOrientation(double headingDegrees, double yawRateDegreesPerSecond) {
        m_orientationPub.set(new double[] {headingDegrees, yawRateDegreesPerSecond, 0, 0, 0, 0});
    }

    /** Returns a new position measurement if the Limelight has seen a tag since the last call. */
    public Optional<PoseMeasurement> getNewPose() {
        TimestampedDoubleArray result = m_botPoseSub.getAtomic();
        double[] data = result.value;
        if (result.timestamp == m_lastTimestampMicros || data.length < kMinLength) {
            return Optional.empty();
        }
        m_lastTimestampMicros = result.timestamp;

        int tagCount = (int) data[kTagCountIndex];
        if (tagCount == 0) {
            return Optional.empty();
        }
        // The picture was taken this long before the result reached the robot
        double timestampSeconds = result.timestamp / 1e6 - data[kLatencyIndex] / 1000.0;
        Pose2d pose = new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));
        int[] tagIds = new int[tagCount];
        for (int i = 0; i < tagCount && kMinLength + i * kValuesPerTag < data.length; i++) {
            tagIds[i] = (int) data[kMinLength + i * kValuesPerTag];
        }
        return Optional.of(new PoseMeasurement(
            pose, timestampSeconds, tagCount, data[kAvgDistanceIndex], tagIds));
    }
}
