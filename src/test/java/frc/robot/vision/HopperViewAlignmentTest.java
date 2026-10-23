package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.Constants.AutoConstants.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        assertEquals(0, kHopperReturnLeftTrimMeters, "Match the hub photo without a separate left trim");
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
