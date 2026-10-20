package frc.robot.vision;

import java.util.Optional;

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

    // Layout of the botpose arrays: x, y, z, roll, pitch, yaw, latency (ms), tag count,
    // tag span, average tag distance, average tag area, then 7 values per tag (ID first).
    private static final int kLatencyIndex = 6;
    private static final int kTagCountIndex = 7;
    private static final int kAvgDistanceIndex = 9;
    private static final int kMinLength = 11;
    private static final int kValuesPerTag = 7;

    private final DoubleArraySubscriber m_botPoseSub;
    private final DoubleArrayPublisher m_orientationPub;
    /** NetworkTables timestamp of the last result returned, so each frame is used only once. */
    private long m_lastTimestampMicros = 0;

    public Limelight(String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        // Robot position in blue-origin field coordinates, i.e. the same as our odometry
        // keepDuplicates: a new frame that happens to give the exact same numbers still counts
        m_botPoseSub = table.getDoubleArrayTopic("botpose_orb_wpiblue")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
        m_orientationPub = table.getDoubleArrayTopic("robot_orientation_set").publish();
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
