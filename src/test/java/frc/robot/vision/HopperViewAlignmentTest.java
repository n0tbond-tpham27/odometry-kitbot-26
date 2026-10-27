package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.vision.Limelight.TargetMeasurement;

class HopperViewAlignmentTest {
    private static final Pose2d GOAL = new Pose2d(3.42587253414264, 4.0346376, Rotation2d.fromDegrees(180));

    @Test
    void photoHasBothTagsLeftAndAboveTheCrosshairAndReproducesTheKnownPose() {
        var photo = HopperViewAlignment.photo();
        assertTrue(photo.mainX() < -15);
        assertTrue(photo.companionX() < photo.mainX() - 10);
        assertTrue(photo.mainY() > 15 && photo.companionY() > 15);
        var sample = sample(photo.mainX(), photo.companionX(), photo.mainY(), photo.companionY());
        assertTrue(photo.matches(sample, 0.001));
        var pose = HopperViewAlignment.estimatePose(sample, photo, GOAL, GOAL.getRotation()).orElseThrow();
        assertEquals(GOAL.getX(), pose.getX(), 1e-9);
        assertEquals(GOAL.getY(), pose.getY(), 1e-9);
        assertEquals(GOAL.getRotation(), pose.getRotation());
        assertTrue(photo.matches(sample, 0.001), "The captured reference stays centered even when the return has a trim");
    }

    @Test
    void shiftedPairViewMeansTwentyCentimetersDriverStationLeftOnEitherAlliance() {
        var reference = HopperViewAlignment.photo();
        var shifted = reference.shiftedDriverStationLeft(kHopperReturnLeftTrimMeters);
        var measurement = sample(shifted.mainX(), shifted.companionX(), shifted.mainY(), shifted.companionY());
        assertFalse(reference.matches(measurement, 1.5));
        assertTrue(shifted.matches(measurement, 1.5));
        for (Pose2d goal : new Pose2d[] {GOAL, new Pose2d(13.1, 4.03, Rotation2d.kZero)}) {
            var pose = HopperViewAlignment.estimatePose(measurement, reference, goal, goal.getRotation()).orElseThrow();
            var error = pose.relativeTo(goal);
            assertEquals(0.0, error.getX(), 1e-9);
            assertEquals(-0.2, error.getY(), 1e-9);
            assertEquals(goal.getRotation(), pose.getRotation());
        }
    }

    @Test
    void imageTranslationDoesNotInheritBiasedFieldRangeOrLateralPositionOnEitherAlliance() {
        var photo = HopperViewAlignment.photo();
        double goalDistance = kHopperPairSpacingMeters
            / (Math.tan(Math.toRadians(photo.mainX())) - Math.tan(Math.toRadians(photo.companionX())));
        double goalRight = -goalDistance * Math.tan(Math.toRadians(photo.mainX()));
        double distance = goalDistance + 0.5;
        double right = goalRight + 0.2;
        double yaw = 15;
        double main = Math.toDegrees(Math.atan(-right / distance)) + yaw;
        double companion = Math.toDegrees(Math.atan(-(right + kHopperPairSpacingMeters) / distance)) + yaw;
        for (Pose2d goal : new Pose2d[] {GOAL, new Pose2d(13.1, 4.03, Rotation2d.kZero)}) {
            Rotation2d gyro = goal.getRotation().plus(Rotation2d.fromDegrees(yaw));
            for (double bias : new double[] {-0.3, 0.0, 0.4}) {
                var sample = new TargetMeasurement(26, right + bias, distance + bias, 10, 45,
                    right + bias, distance + bias, main, companion, true);
                var pose = HopperViewAlignment.estimatePose(sample, photo, goal, gyro).orElseThrow();
                Translation2d expected = goal.getTranslation().plus(new Translation2d(0.5, 0.2).rotateBy(goal.getRotation()));
                assertEquals(expected.getX(), pose.getX(), 1e-9);
                assertEquals(expected.getY(), pose.getY(), 1e-9);
                assertEquals(gyro, pose.getRotation());
            }
        }
    }

    @Test
    void pairSpacingMatchesBothHubFacesInTheOfficialLayout() throws Exception {
        var layout = new ObjectMapper().readTree(getClass().getResourceAsStream(
            "/edu/wpi/first/apriltag/2026-rebuilt-welded.json"));
        for (int[] pair : new int[][] {{26, 25}, {10, 9}}) {
            com.fasterxml.jackson.databind.JsonNode main = null;
            com.fasterxml.jackson.databind.JsonNode companion = null;
            for (var tag : layout.get("tags")) {
                if (tag.get("ID").asInt() == pair[0]) { main = tag.get("pose").get("translation"); }
                if (tag.get("ID").asInt() == pair[1]) { companion = tag.get("pose").get("translation"); }
            }
            assertNotNull(main);
            assertNotNull(companion);
            assertEquals(main.get("x").asDouble(), companion.get("x").asDouble(), 1e-9);
            assertEquals(main.get("z").asDouble(), companion.get("z").asDouble(), 1e-9);
            assertEquals(kHopperPairSpacingMeters,
                Math.abs(main.get("y").asDouble() - companion.get("y").asDouble()), 1e-9);
        }
    }

    @Test
    void aCenteredPrimaryTagDoesNotCountAsThePhotographedView() {
        var photo = HopperViewAlignment.photo();
        assertFalse(photo.matches(sample(0, -15, photo.mainY(), photo.companionY()), 1.5));
        assertFalse(photo.matches(sample(photo.mainX(), photo.companionX(), 0, 0), 1.5));
        assertFalse(photo.matches(sample(photo.mainX(), Double.NaN, photo.mainY(), photo.companionY()), 1.5));
    }

    @Test
    void gyroYawIsRemovedFromTheTranslationEstimateAndNeverReplacedByCameraYaw() {
        var photo = HopperViewAlignment.photo();
        var sample = sample(photo.mainX() + 4, photo.companionX() + 4, photo.mainY(), photo.companionY());
        Rotation2d gyro = GOAL.getRotation().plus(Rotation2d.fromDegrees(4));
        var pose = HopperViewAlignment.estimatePose(sample, photo, GOAL, gyro).orElseThrow();
        assertEquals(GOAL.getX(), pose.getX(), 1e-9);
        assertEquals(GOAL.getY(), pose.getY(), 1e-9);
        assertEquals(gyro, pose.getRotation());
        assertFalse(photo.matches(sample, 1.5), "Even a correct translation still needs the gyro to square the robot");
        assertTrue(HopperViewAlignment.estimatePose(sample, photo, GOAL,
            GOAL.getRotation().plus(Rotation2d.fromDegrees(60))).isEmpty());
    }

    private static TargetMeasurement sample(double mainX, double companionX, double mainY, double companionY) {
        return new TargetMeasurement(26, 0, 1.05, 10, 45, 0, 1.05,
            mainX, companionX, true, mainY, companionY);
    }
}
