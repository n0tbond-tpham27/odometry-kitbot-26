package frc.robot.vision;

import java.util.Optional;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.OdometryConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        Pose2d pose, double timestampSeconds, int tagCount, double avgTagDistanceMeters, int[] tagIds,
        double[] tagHorizontalDegrees) {
        public PoseMeasurement(Pose2d pose, double timestampSeconds, int tagCount,
                double avgTagDistanceMeters, int[] tagIds) {
            this(pose, timestampSeconds, tagCount, avgTagDistanceMeters, tagIds, new double[0]);
        }
    }

    /** Robot and camera positions relative to the selected tag's center, independent of odometry. */
    public record TargetMeasurement(int tagId, double rightMeters, double outwardMeters, double timestampSeconds,
            double fieldHeadingDegrees, double cameraRightMeters, double cameraOutwardMeters,
            double imageHorizontalDegrees) {
        public TargetMeasurement(int tagId, double rightMeters, double outwardMeters, double timestampSeconds,
                double fieldHeadingDegrees, double cameraRightMeters, double cameraOutwardMeters) {
            this(tagId, rightMeters, outwardMeters, timestampSeconds, fieldHeadingDegrees,
                cameraRightMeters, cameraOutwardMeters, Double.NaN);
        }
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
    private final DoubleArraySubscriber m_cameraRobotPoseSub;
    private Translation2d m_cameraOffsetRobot = Translation2d.kZero;
    private String m_hopperSource = "waiting for vision";
    private final edu.wpi.first.networktables.DoublePublisher m_priorityPub;
    private long m_lastTargetTimestampMicros = 0;
    /** NetworkTables timestamp of the last result returned, so each frame is used only once. */
    private long m_lastTimestampMicros = 0;

    public Limelight(String name) {
        this(NetworkTableInstance.getDefault(), name);
    }

    Limelight(NetworkTableInstance instance, String name) {
        NetworkTable table = instance.getTable(name);
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
        m_cameraRobotPoseSub = table.getDoubleArrayTopic("camerapose_robotspace").subscribe(new double[0]);
        m_priorityPub = table.getDoubleTopic("priorityid").publish();
    }

    /** Prefer one complete MegaTag2 frame; single-target PnP remains a fallback. */
    public Optional<TargetMeasurement> getNewHopperTarget(int tagId, Pose2d goal, double gyroHeadingDegrees, double now) {
        double[] mount = m_cameraRobotPoseSub.get();
        if (mount.length >= 6 && Double.isFinite(mount[0]) && Double.isFinite(mount[1])) {
            // Limelight robot-space Y is right-positive; WPILib robot-space Y is left-positive.
            m_cameraOffsetRobot = new Translation2d(mount[0], -mount[1]);
        }
        Optional<PoseMeasurement> field = getNewPose();
        if (field.isPresent()) {
            Optional<TargetMeasurement> target = targetFromFieldPose(field.get(), tagId, goal,
                gyroHeadingDegrees, m_cameraOffsetRobot, now);
            if (target.isPresent()) {
                m_hopperSource = "MegaTag2 field pose " + java.util.Arrays.toString(field.get().tagIds());
                // Older firmware may omit the per-tag block. A matching t2d capture still
                // supplies the selected tag's image position without delaying localization.
                if (!Double.isFinite(target.get().imageHorizontalDegrees())) {
                    TimestampedDoubleArray stats = m_targetStatsSub.getAtomic();
                    double angle = imageAngleFromStats(stats.value, stats.timestamp,
                        m_lastTimestampMicros, tagId);
                    return Optional.of(withImageAngle(target.get(), angle));
                }
                return target;
            }
        }
        Optional<TargetMeasurement> direct = getNewTargetPose(tagId);
        m_hopperSource = direct.isPresent() ? "tag-relative PnP" : "waiting for usable field or tag pose";
        return direct;
    }

    public String getHopperSource() {
        return m_hopperSource;
    }

    static Optional<TargetMeasurement> targetFromFieldPose(PoseMeasurement frame, int tagId, Pose2d goal,
            double headingDegrees, Translation2d cameraOffsetRobot, double now) {
        double age = now - frame.timestampSeconds();
        if (frame.tagCount() < 1 || !Double.isFinite(age) || age > kMaxVisionAgeSeconds
                || age < -kVisionFutureToleranceSeconds || !Double.isFinite(headingDegrees)
                || !Double.isFinite(frame.pose().getX()) || !Double.isFinite(frame.pose().getY())
                || !Double.isFinite(frame.avgTagDistanceMeters()) || frame.avgTagDistanceMeters() < 0.0
                || frame.avgTagDistanceMeters() > kMaxTagDistanceMeters
                || frame.pose().getX() < 0.0 || frame.pose().getX() > kFieldLengthMeters
                || frame.pose().getY() < 0.0 || frame.pose().getY() > kFieldWidthMeters) {
            return Optional.empty();
        }
        Pose2d robot = new Pose2d(frame.pose().getTranslation(), Rotation2d.fromDegrees(headingDegrees));
        Pose2d relative = robot.relativeTo(goal);
        Pose2d camera = new Pose2d(robot.getTranslation().plus(cameraOffsetRobot.rotateBy(robot.getRotation())),
            robot.getRotation()).relativeTo(goal);
        double right = relative.getY() + kHopperPathRightReferenceMeters;
        double outward = relative.getX() + kHopperTagStandOffMeters;
        if (Math.abs(right) > kHopperTagMaxLateralMeters || outward <= 0.1 || outward > kHopperTagMaxRangeMeters) {
            return Optional.empty();
        }
        // The field pose can use any mapped tag, keeping feedback alive when tag 26 leaves
        // the image during a correction. tagId identifies the target reference, not the observed IDs.
        double imageAngle = Double.NaN;
        for (int i = 0; i < frame.tagIds().length && i < frame.tagHorizontalDegrees().length; i++) {
            if (frame.tagIds()[i] == tagId && validImageAngle(frame.tagHorizontalDegrees()[i])) {
                imageAngle = frame.tagHorizontalDegrees()[i];
                break;
            }
        }
        return Optional.of(new TargetMeasurement(tagId, right, outward, frame.timestampSeconds(), headingDegrees,
            camera.getY() + kHopperPathRightReferenceMeters, camera.getX() + kHopperTagStandOffMeters, imageAngle));
    }

    private static boolean validImageAngle(double angle) {
        return Double.isFinite(angle) && Math.abs(angle) < 70.0;
    }

    private static double imageAngleFromStats(double[] stats, long timestamp, long poseTimestamp, int tagId) {
        return stats.length >= 10 && timestamp > 0 && Math.abs(timestamp - poseTimestamp) <= 20000
            && stats[0] == 1.0 && stats[9] == tagId && validImageAngle(stats[6]) ? stats[6] : Double.NaN;
    }

    private static TargetMeasurement withImageAngle(TargetMeasurement target, double angle) {
        return new TargetMeasurement(target.tagId(), target.rightMeters(), target.outwardMeters(),
            target.timestampSeconds(), target.fieldHeadingDegrees(), target.cameraRightMeters(),
            target.cameraOutwardMeters(), angle);
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
            valid ? cameraPose[0] : Double.NaN, valid ? cameraPose[2] : Double.NaN, measurement.imageHorizontalDegrees());
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
            measurement.cameraRightMeters(), measurement.cameraOutwardMeters(), measurement.imageHorizontalDegrees());
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
        return Optional.of(new TargetMeasurement(requiredTagId, pose[0], pose[2], captureSeconds,
            Double.NaN, Double.NaN, Double.NaN,
            imageAngleFromStats(stats, statsTimestampMicros, poseTimestampMicros, requiredTagId)));
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

        if (!Double.isFinite(data[kTagCountIndex]) || data[kTagCountIndex] < 1.0 || data[kTagCountIndex] > 64.0
                || !Double.isFinite(data[kLatencyIndex]) || data[kLatencyIndex] < 0.0
                || !Double.isFinite(data[0]) || !Double.isFinite(data[1]) || !Double.isFinite(data[5])) {
            return Optional.empty();
        }
        int tagCount = (int) data[kTagCountIndex];
        // The picture was taken this long before the result reached the robot
        double timestampSeconds = result.timestamp / 1e6 - data[kLatencyIndex] / 1000.0;
        Pose2d pose = new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));
        int[] tagIds = new int[tagCount];
        double[] tagHorizontalDegrees = new double[tagCount];
        java.util.Arrays.fill(tagHorizontalDegrees, Double.NaN);
        for (int i = 0; i < tagCount && kMinLength + i * kValuesPerTag < data.length; i++) {
            tagIds[i] = (int) data[kMinLength + i * kValuesPerTag];
            if (kMinLength + i * kValuesPerTag + 1 < data.length) {
                tagHorizontalDegrees[i] = data[kMinLength + i * kValuesPerTag + 1];
            }
        }
        return Optional.of(new PoseMeasurement(
            pose, timestampSeconds, tagCount, data[kAvgDistanceIndex], tagIds, tagHorizontalDegrees));
    }
}
