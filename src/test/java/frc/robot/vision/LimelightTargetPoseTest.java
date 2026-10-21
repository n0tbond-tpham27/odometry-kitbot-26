package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class LimelightTargetPoseTest {
    private static double[] stats(int id) {
        return new double[] {1, 1, 20, 10, 0, 0, 0, 0, 5, id};
    }

    @Test
    void decodesTag26PositionAndCaptureLatency() {
        var result = Limelight.parseTargetPose(new double[] {0.25, 0.5, 0.6, 0, 0, 0},
            10_000_000, stats(26), 10_001_000, 26).orElseThrow();
        assertEquals(26, result.tagId());
        assertEquals(0.25, result.rightMeters());
        assertEquals(0.6, result.outwardMeters());
        assertEquals(9.97, result.timestampSeconds(), 1e-9);
    }

    @Test
    void rejectsWrongTagMismatchedFramesAndInvalidData() {
        double[] pose = {0.2, 0.5, 0.6, 0, 0, 0};
        assertTrue(Limelight.parseTargetPose(pose, 10_000_000, stats(25), 10_000_000, 26).isEmpty());
        assertTrue(Limelight.parseTargetPose(pose, 10_000_000, stats(26), 10_050_000, 26).isEmpty());
        double[] invalid = stats(26);
        invalid[0] = 0;
        assertTrue(Limelight.parseTargetPose(pose, 10_000_000, invalid, 10_000_000, 26).isEmpty());
        invalid = stats(26);
        invalid[2] = Double.NaN;
        assertTrue(Limelight.parseTargetPose(pose, 10_000_000, invalid, 10_000_000, 26).isEmpty());
        assertTrue(Limelight.parseTargetPose(new double[6], 10_000_000, stats(26), 10_000_000, 26).isEmpty());
    }

    @Test
    void onlySeedsHeadingFromMatchingUnambiguousTagFrames() {
        var measurement = new Limelight.TargetMeasurement(26, 0.2, 0.6, 9.97);
        double[] fieldPose = new double[18];
        fieldPose[5] = 180;
        fieldPose[7] = 1;
        fieldPose[17] = 0.1;
        assertEquals(180, Limelight.withFieldHeading(measurement, 10_000_000, fieldPose, 10_001_000)
            .fieldHeadingDegrees());
        fieldPose[17] = 0.9;
        assertTrue(Double.isNaN(Limelight.withFieldHeading(measurement, 10_000_000, fieldPose, 10_001_000)
            .fieldHeadingDegrees()));
        fieldPose[17] = 0.1;
        assertTrue(Double.isNaN(Limelight.withFieldHeading(measurement, 10_000_000, fieldPose, 9_900_000)
            .fieldHeadingDegrees()));
    }

    @Test
    void readsSeparateCameraPositionAndRejectsMismatchedCameraFrames() {
        var robot = new Limelight.TargetMeasurement(26, 0.2, 0.6, 9.97, 180);
        double[] cameraPose = {0.12, 0.3, 0.4, 0, 0, 0};
        var result = Limelight.withCameraPose(robot, 10_000_000, cameraPose, 10_001_000);
        assertEquals(0.12, result.cameraRightMeters());
        assertEquals(0.4, result.cameraOutwardMeters());
        assertEquals(0.2, result.rightMeters());
        assertEquals(0.6, result.outwardMeters());
        assertTrue(Double.isNaN(Limelight.withCameraPose(robot, 10_000_000, cameraPose, 9_900_000)
            .cameraRightMeters()));
    }
}
