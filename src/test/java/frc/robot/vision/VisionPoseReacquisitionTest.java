package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.Limelight.PoseMeasurement;

class VisionPoseReacquisitionTest {
    private static final Pose2d ODOMETRY = new Pose2d(5.0, 4.0, Rotation2d.kZero);

    private static VisionPoseReacquisition collector() {
        VisionPoseReacquisition collector = new VisionPoseReacquisition(5, 0.12, 2.0);
        collector.reset(10.0);
        return collector;
    }

    @Test
    void reacquiresLargeDriftOnlyFromFiveAgreeingMultiTagCaptures() {
        VisionPoseReacquisition collector = collector();
        for (int i = 1; i <= 5; i++) {
            var result = collector.add(frame(6.4 + i * 0.002, 4.1, 10.0 + i * 0.04, 2),
                ODOMETRY, 10.02 + i * 0.04);
            assertEquals(i == 5, result.isPresent());
        }
        Translation2d result = collector.getPosition(10.22).orElseThrow();
        assertEquals(6.406, result.getX(), 1e-9);
        assertEquals(4.1, result.getY(), 1e-9);
    }

    @Test
    void rejectsPreStopRepeatedStaleAndFutureCaptures() {
        VisionPoseReacquisition collector = collector();
        collector.add(frame(5.1, 4.0, 9.99, 2), ODOMETRY, 10.02);
        collector.add(frame(5.1, 4.0, 10.01, 2), ODOMETRY, 10.5);
        collector.add(frame(5.1, 4.0, 10.6, 2), ODOMETRY, 10.1);
        assertEquals(0, collector.getFrameCount());
        collector.add(frame(5.1, 4.0, 10.1, 2), ODOMETRY, 10.12);
        for (int i = 0; i < 10; i++) {
            assertTrue(collector.add(frame(5.1, 4.0, 10.1, 2), ODOMETRY, 10.12).isEmpty());
        }
        assertEquals(1, collector.getFrameCount());
    }

    @Test
    void rejectsSingleTagLargeJumpsAndOutOfBoundsMeasurements() {
        VisionPoseReacquisition collector = collector();
        collector.add(frame(6.4, 4.0, 10.04, 1), ODOMETRY, 10.06);
        collector.add(frame(7.1, 4.0, 10.08, 2), ODOMETRY, 10.10);
        collector.add(frame(5.0, -0.1, 10.12, 2), ODOMETRY, 10.14);
        collector.add(frame(Double.NaN, 4.0, 10.16, 2), ODOMETRY, 10.18);
        collector.add(new PoseMeasurement(ODOMETRY, 10.2, 0, 2.0, new int[0]), ODOMETRY, 10.22);
        collector.add(new PoseMeasurement(ODOMETRY, 10.24, 2, 5.0, new int[] {1, 2}), ODOMETRY, 10.26);
        collector.add(new PoseMeasurement(ODOMETRY, 10.28, 2, 2.0, new int[] {1, 0}), ODOMETRY, 10.30);
        collector.add(new PoseMeasurement(ODOMETRY, 10.32, 2, 2.0, new int[] {1, 1}), ODOMETRY, 10.34);
        collector.add(new PoseMeasurement(ODOMETRY, 10.36, 2, 2.0, new int[] {1}), ODOMETRY, 10.38);
        assertEquals(0, collector.getFrameCount());
    }

    @Test
    void singleTagCanRecoverSmallDrift() {
        VisionPoseReacquisition collector = collector();
        for (int i = 1; i <= 5; i++) {
            collector.add(frame(5.2, 4.0, 10.0 + i * 0.04, 1), ODOMETRY, 10.02 + i * 0.04);
        }
        assertTrue(collector.getPosition(10.22).isPresent());
    }

    @Test
    void startsNewClusterOnOutlierOrCameraGapAndExpiresOldSolution() {
        VisionPoseReacquisition collector = collector();
        collector.add(frame(5.1, 4.0, 10.04, 2), ODOMETRY, 10.06);
        collector.add(frame(5.1, 4.0, 10.08, 2), ODOMETRY, 10.10);
        collector.add(frame(5.4, 4.0, 10.12, 2), ODOMETRY, 10.14);
        assertEquals(1, collector.getFrameCount());
        collector.add(frame(5.4, 4.0, 10.5, 2), ODOMETRY, 10.52);
        assertEquals(1, collector.getFrameCount());
        for (int i = 1; i <= 4; i++) {
            collector.add(frame(5.4, 4.0, 10.5 + i * 0.04, 2), ODOMETRY, 10.52 + i * 0.04);
        }
        assertTrue(collector.getPosition(10.68).isPresent());
        assertTrue(collector.getPosition(11.0).isEmpty());
        collector.reset(11.0);
        assertEquals(0, collector.getFrameCount());
        assertTrue(collector.getPosition(11.0).isEmpty());
    }

    private static PoseMeasurement frame(double x, double y, double time, int tags) {
        return new PoseMeasurement(new Pose2d(x, y, Rotation2d.fromDegrees(180)), time, tags,
            2.0, tags == 1 ? new int[] {1} : new int[] {1, 2});
    }
}
