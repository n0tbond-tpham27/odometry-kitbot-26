package frc.robot.vision;

import static frc.robot.Constants.OdometryConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.HopperTagAlignmentCommand;
import frc.robot.vision.Limelight.PoseMeasurement;

class LimelightHopperTargetTest {
    private static final Pose2d GOAL = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));

    @BeforeAll
    static void initializeNativeLibraries() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void usesBothHopperTagsAndIndependentHeadingEvenWhenSuppliedGyroHeadingIsWrong() {
        try (var instance = NetworkTableInstance.create()) {
            var limelight = new Limelight(instance, "test");
            var table = instance.getTable("test");
            try (var paired = table.getDoubleArrayTopic("botpose_wpiblue").publish();
                    var gyroBased = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish()) {
                double[] frame = pairedFrame(25, 26);
                frame[0] -= 0.6;
                frame[1] -= 0.25;
                frame[5] = 167;
                paired.set(frame, 10_000_000);
                gyroBased.set(new double[] {GOAL.getX(), GOAL.getY(), 0, 0, 0, 180, 40, 1, 0, 1, 0.2}, 10_000_000);
                var target = limelight.getNewHopperTarget(26, GOAL, 180, 10.02).orElseThrow();
                assertTrue(target.hasHopperPair());
                assertEquals(167, target.fieldHeadingDegrees(), 1e-9);
                assertEquals(-6, target.imageHorizontalDegrees(), 1e-9);
                assertEquals(-31, target.companionImageHorizontalDegrees(), 1e-9);
                assertEquals(0.25, target.rightMeters(), 1e-9);
                assertEquals(GOAL.getX() - 0.6, target.toFieldPose(GOAL).getX(), 1e-9);
                assertTrue(limelight.getHopperSource().contains("25 + 26"));
                // An MT2 update cannot masquerade as another independent two-tag capture.
                assertFalse(limelight.getNewHopperTarget(26, GOAL, 180, 10.02).orElseThrow().hasHopperPair());
                assertTrue(limelight.getNewHopperTarget(26, GOAL, 180, 10.02).isEmpty());
            }
        }
    }

    @Test
    void pairedPoseRequiresTheCorrectTwoIdsInOneFreshCompleteFrameOnEitherAlliance() {
        for (int primary : new int[] {26, 10}) {
            Pose2d goal = primary == 26 ? GOAL : new Pose2d(13.1, 4.0346376, Rotation2d.kZero);
            double[] frame = pairedFrame(Limelight.companionTagId(primary), primary);
            frame[0] = goal.getX();
            frame[1] = goal.getY();
            frame[5] = goal.getRotation().getDegrees();
            assertTrue(Limelight.pairedTargetFromFrame(frame, 10_000_000, primary, goal,
                Translation2d.kZero, 10.02).orElseThrow().hasHopperPair());
            assertTrue(Limelight.pairedTargetFromFrame(frame, 9_000_000, primary, goal,
                Translation2d.kZero, 10.02).isEmpty());
            assertTrue(Limelight.pairedTargetFromFrame(frame, 11_000_000, primary, goal,
                Translation2d.kZero, 10.02).isEmpty());
            for (int scenario = 0; scenario < 6; scenario++) {
                double[] invalid = frame.clone();
                switch (scenario) {
                    case 0 -> invalid[11] = primary; // Two copies of one tag do not form a pair.
                    case 1 -> invalid[11] = 24; // Two tags, but the required companion is missing.
                    case 2 -> invalid[17] = 0.8; // Ambiguous companion.
                    case 3 -> invalid[12] = Double.NaN;
                    case 4 -> invalid[12] = invalid[19]; // Degenerate angular separation.
                    case 5 -> invalid = java.util.Arrays.copyOf(invalid, 24); // Incomplete raw block.
                }
                assertTrue(Limelight.pairedTargetFromFrame(invalid, 10_000_000, primary, goal,
                    Translation2d.kZero, 10.02).isEmpty(), "Invalid paired frame scenario " + scenario);
            }
        }
    }

    private static double[] pairedFrame(int companion, int primary) {
        return new double[] {GOAL.getX(), GOAL.getY(), 0, 0, 0, 180, 40, 2, 0.3556, 1, 0.2,
            companion, -31, 0, 0.1, 1, 1, 0.1,
            primary, -6, 0, 0.1, 1, 1, 0.1};
    }

    @Test
    void acquiresFromOneAtomicFieldFrameWithoutOtherLimelightStreams() {
        try (var instance = NetworkTableInstance.create()) {
            var limelight = new Limelight(instance, "test");
            var table = instance.getTable("test");
            try (var publisher = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish()) {
                // Only the standard MegaTag2 header is present: no t2d, targetspace,
                // camera targetspace, MegaTag1, or optional raw fiducial arrays.
                publisher.set(new double[] {GOAL.getX() - 0.8, GOAL.getY() - 0.3,
                    0, 0, 0, 180, 40, 1, 0, 1.5, 0.2}, 10_000_000);
                var target = limelight.getNewHopperTarget(26, GOAL, 180, 10.02).orElseThrow();
                assertEquals(9.96, target.timestampSeconds(), 1e-9);
                assertEquals(0.3, target.rightMeters(), 1e-9);
                assertEquals(kHopperTagStandOffMeters + 0.8, target.outwardMeters(), 1e-9);
                assertEquals(GOAL.getY() - 0.3,
                    HopperTagAlignmentCommand.fieldPoseFromTag(target, GOAL, GOAL).getY(), 1e-9);
                assertTrue(limelight.getNewHopperTarget(26, GOAL, 180, 10.02).isEmpty(),
                    "A frame must be used only once");
            }
        }
    }

    @Test
    void anyMappedTagCanMaintainFeedbackForTheTag26ReturnReference() {
        Pose2d actual = new Pose2d(GOAL.getX() - 0.8, GOAL.getY() - 0.3, GOAL.getRotation());
        var frame = new PoseMeasurement(actual, 10, 1, 1.5, new int[] {25});
        var target = Limelight.targetFromFieldPose(frame, 26, GOAL, 180, Translation2d.kZero, 10.04)
            .orElseThrow();
        assertEquals(26, target.tagId());
        Pose2d reconstructed = HopperTagAlignmentCommand.fieldPoseFromTag(target, GOAL, GOAL);
        assertEquals(actual.getX(), reconstructed.getX(), 1e-9);
        assertEquals(actual.getY(), reconstructed.getY(), 1e-9);
    }

    @Test
    void preservesMountedCameraOffsetAndAllianceCoordinateSigns() {
        for (Pose2d goal : new Pose2d[] {GOAL, new Pose2d(13.1, 3.7, Rotation2d.kZero)}) {
            var frame = new PoseMeasurement(goal, 10, 1, 1, new int[] {26});
            var target = Limelight.targetFromFieldPose(frame, 26, goal, goal.getRotation().getDegrees(),
                new Translation2d(-0.2, 0.08), 10.02).orElseThrow();
            assertEquals(kHopperPathRightReferenceMeters, target.rightMeters(), 1e-9);
            assertEquals(kHopperPathRightReferenceMeters + 0.08, target.cameraRightMeters(), 1e-9);
            assertEquals(kHopperTagStandOffMeters - 0.2, target.cameraOutwardMeters(), 1e-9);
        }
    }

    @Test
    void readsCameraMountFromNetworkTablesIndependentlyOfCaptureTimestamp() {
        try (var instance = NetworkTableInstance.create()) {
            var limelight = new Limelight(instance, "test");
            var table = instance.getTable("test");
            try (var mount = table.getDoubleArrayTopic("camerapose_robotspace").publish();
                    var field = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish()) {
                mount.set(new double[] {-0.2, -0.08, 0.5, 0, 0, 180}, 1_000_000);
                field.set(new double[] {GOAL.getX(), GOAL.getY(), 0, 0, 0, 180, 20, 1, 0, 1, 0.2}, 10_000_000);
                var target = limelight.getNewHopperTarget(26, GOAL, 180, 10.02).orElseThrow();
                assertEquals(kHopperPathRightReferenceMeters + 0.08, target.cameraRightMeters(), 1e-9);
                assertEquals(kHopperTagStandOffMeters - 0.2, target.cameraOutwardMeters(), 1e-9);
            }
        }
    }

    @Test
    void rejectsStaleFutureTaglessAndOutOfBoundsPositions() {
        for (PoseMeasurement frame : new PoseMeasurement[] {
                new PoseMeasurement(GOAL, 9, 1, 1, new int[] {26}),
                new PoseMeasurement(GOAL, 11, 1, 1, new int[] {26}),
                new PoseMeasurement(GOAL, 10, 0, 1, new int[0]),
                new PoseMeasurement(new Pose2d(-1, 4, GOAL.getRotation()), 10, 1, 1, new int[] {26}),
                new PoseMeasurement(GOAL, 10, 1, Double.NaN, new int[] {26})}) {
            assertTrue(Limelight.targetFromFieldPose(frame, 26, GOAL, 180, Translation2d.kZero, 10.02).isEmpty());
        }
    }

    @Test
    void readsTag26ImageOffsetFromItsOwnFiducialInTheAtomicFieldFrame() {
        try (var instance = NetworkTableInstance.create()) {
            var limelight = new Limelight(instance, "test");
            try (var publisher = instance.getTable("test").getDoubleArrayTopic("botpose_orb_wpiblue").publish()) {
                publisher.set(new double[] {GOAL.getX(), GOAL.getY(), 0, 0, 0, 180, 40, 2, 0.4, 1, 0.2,
                    25, 0, 0, 0.1, 1, 1, 0.1,
                    26, -6, 1, 0.1, 1, 1, 0.1}, 10_000_000);
                var target = limelight.getNewHopperTarget(26, GOAL, 180, 10.02).orElseThrow();
                assertEquals(-6, target.imageHorizontalDegrees(), 1e-9,
                    "The centered tag 25 must not be mistaken for the desired tag 26");
                assertEquals(9.96, target.timestampSeconds(), 1e-9);
            }
        }
    }

    @Test
    void usesOnlyMatchingTagAndTimestampWhenReadingImageOffsetFromT2d() {
        for (int scenario = 0; scenario < 3; scenario++) {
            try (var instance = NetworkTableInstance.create()) {
                var limelight = new Limelight(instance, "test");
                var table = instance.getTable("test");
                try (var field = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish();
                        var stats = table.getDoubleArrayTopic("t2d").publish()) {
                    field.set(new double[] {GOAL.getX(), GOAL.getY(), 0, 0, 0, 180, 20, 1, 0, 1, 0.2}, 10_000_000);
                    stats.set(new double[] {1, 1, 10, 10, 20, 0, -5, 0, 0.2, scenario == 1 ? 25 : 26},
                        scenario == 2 ? 9_900_000 : 10_001_000);
                    var target = limelight.getNewHopperTarget(26, GOAL, 180, 10.02).orElseThrow();
                    if (scenario == 0) {
                        assertEquals(-5, target.imageHorizontalDegrees(), 1e-9,
                            "Use principal-pixel offset, independent of crosshair calibration");
                    } else {
                        assertTrue(Double.isNaN(target.imageHorizontalDegrees()),
                            "A different tag or old frame must not confirm image centering");
                    }
                }
            }
        }
    }
}
