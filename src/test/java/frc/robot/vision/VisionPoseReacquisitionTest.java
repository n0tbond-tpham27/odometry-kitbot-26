package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.vision.Limelight.PoseMeasurement;

class VisionPoseReacquisitionTest {
    private static final Pose2d ODOMETRY = new Pose2d(5.0, 4.0, Rotation2d.kZero);

    private static VisionPoseReacquisition collector() {
        VisionPoseReacquisition collector = new VisionPoseReacquisition(5, 0.12, 2.0);
        collector.reset(10.0);
        return collector;
    }

    @Test
    void eachTrenchTagCanRecoverLargeDriftFromFiveReliableSingleTagCaptures() {
        for (int id : new int[] {1, 6, 7, 12, 17, 22, 23, 28}) {
            var collector = VisionPoseReacquisition.forTrenchTags();
            collector.reset(10);
            for (int i = 1; i <= 5; i++) {
                var result = collector.add(trenchFrame(6.4, 4.1, 10 + i * 0.04, id, 0.05), ODOMETRY,
                    10.02 + i * 0.04);
                assertEquals(i == 5, result.isPresent(), "Single trench tag " + id);
            }
            assertEquals(6.4, collector.getPosition(10.22).orElseThrow().getX(), 1e-9);
        }
    }

    @Test
    void movingRobotComparesOffsetsAtCaptureTimeInsteadOfRequiringStationaryFieldPositions() {
        var collector = VisionPoseReacquisition.forTrenchTags();
        collector.reset(10);
        for (int i = 1; i <= 5; i++) {
            Pose2d historical = new Pose2d(5 + i * 0.25, 4, Rotation2d.fromDegrees(32));
            collector.add(trenchFrame(historical.getX() + 0.45, historical.getY() - 0.3,
                10 + i * 0.04, 28, 0.05), historical, 10.12 + i * 0.04);
        }
        var result = collector.getMeasurement(10.32).orElseThrow();
        assertEquals(6.70, result.pose().getX(), 1e-9);
        assertEquals(3.70, result.pose().getY(), 1e-9);
        assertEquals(32, result.pose().getRotation().getDegrees(), 1e-9,
            "Use the gyro heading at capture, not the camera's yaw");
        assertEquals(10.20, result.timestampSeconds(), 1e-9);
        assertEquals(5, collector.getFrameCount());
    }

    @Test
    void trenchCorrectionPreservesTravelDuringCameraLatencyAndDoesNotRotateTheRobotPose() {
        var estimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0.6),
            Rotation2d.kZero, 0, 0, new Pose2d(5, 4, Rotation2d.fromDegrees(20)),
            VecBuilder.fill(0.02, 0.02, 0.02), VecBuilder.fill(0.7, 0.7, 9999999));
        for (int i = 0; i <= 15; i++) {
            estimator.updateWithTime(10 + i * 0.02, Rotation2d.kZero, i * 0.04, i * 0.04);
        }
        var collector = VisionPoseReacquisition.forTrenchTags();
        collector.reset(10);
        for (int i = 1; i <= 5; i++) {
            double capture = 10 + i * 0.04;
            Pose2d historical = estimator.sampleAt(capture).orElseThrow();
            collector.add(trenchFrame(historical.getX() + 0.45, historical.getY() - 0.3,
                capture, 22, 0.05), historical, capture + 0.10);
        }
        var accepted = collector.getMeasurement(10.30).orElseThrow();
        Pose2d before = estimator.getEstimatedPosition();
        estimator.addVisionMeasurement(accepted.pose(), accepted.timestampSeconds(),
            VecBuilder.fill(0.01, 0.01, 9999999));
        Pose2d after = estimator.getEstimatedPosition();
        assertEquals(before.getX() + 0.30, after.getX(), 1e-6);
        assertEquals(before.getY() - 0.20, after.getY(), 1e-6);
        assertEquals(before.getRotation().getRadians(), after.getRotation().getRadians(), 1e-9);
    }

    @Test
    void trenchModeRejectsHubTagsUnknownQualityLongRangeAndExcessiveCorrections() {
        var collector = VisionPoseReacquisition.forTrenchTags();
        collector.reset(10);
        for (int id : new int[] {25, 26, 9, 10, 18, 99}) {
            collector.add(trenchFrame(5.2, 4, 10.04, id, 0.05), ODOMETRY, 10.06);
        }
        for (double ambiguity : new double[] {-0.1, 0.21, Double.NaN, Double.POSITIVE_INFINITY}) {
            collector.add(trenchFrame(5.2, 4, 10.04, 17, ambiguity), ODOMETRY, 10.06);
        }
        collector.add(new PoseMeasurement(ODOMETRY, 10.04, 1, 2, new int[] {17}), ODOMETRY, 10.06);
        collector.add(new PoseMeasurement(ODOMETRY, 10.04, 1, 3.5, new int[] {17},
            new double[] {0}, new double[] {0.05}), ODOMETRY, 10.06);
        collector.add(trenchFrame(7.1, 4, 10.04, 17, 0.05), ODOMETRY, 10.06);
        assertEquals(0, collector.getFrameCount());
        assertTrue(collector.getMeasurement(10.06).isEmpty());
    }

    @Test
    void trenchModeRejectsStaleCapturesAndStartsOverForAnInconsistentOffset() {
        var collector = VisionPoseReacquisition.forTrenchTags();
        collector.reset(10);
        collector.add(trenchFrame(5.2, 4, 9.99, 17, 0.05), ODOMETRY, 10.02);
        collector.add(trenchFrame(5.2, 4, 10.04, 17, 0.05), ODOMETRY, 10.5);
        assertEquals(0, collector.getFrameCount());
        collector.add(trenchFrame(5.2, 4, 10.04, 17, 0.05), ODOMETRY, 10.06);
        collector.add(trenchFrame(5.2, 4, 10.04, 17, 0.05), ODOMETRY, 10.06);
        assertEquals(1, collector.getFrameCount());
        collector.add(trenchFrame(5.2, 4, 10.08, 17, 0.05), ODOMETRY, 10.10);
        collector.add(trenchFrame(5.5, 4, 10.12, 17, 0.05), ODOMETRY, 10.14);
        assertEquals(1, collector.getFrameCount());
        assertTrue(collector.getMeasurement(10.14).isEmpty());
    }

    private static PoseMeasurement trenchFrame(double x, double y, double time, int id, double ambiguity) {
        return new PoseMeasurement(new Pose2d(x, y, Rotation2d.fromDegrees(90)), time,
            1, 2.0, new int[] {id}, new double[] {0}, new double[] {ambiguity});
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
