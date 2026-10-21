package frc.robot.vision;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.OdometryConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class HopperAlignmentControllerTest {
    private static final double DT = 0.02;
    private static final Pose2d TARGET = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));

    @Test
    void centersFromEitherSideAndDifferentHeadings() {
        for (double xOffset : new double[] {-0.6, 0.0, 0.25}) {
            for (double yOffset : new double[] {-0.6, 0.0, 0.6}) {
                for (double heading : new double[] {140, 180, 220}) {
                    assertConverges(new Pose2d(TARGET.getX() + xOffset, TARGET.getY() + yOffset,
                        Rotation2d.fromDegrees(heading)), TARGET);
                }
            }
        }
    }

    @Test
    void centersOnRedSideWithOppositeFinalHeading() {
        Pose2d redTarget = new Pose2d(13.1, 4.1015, Rotation2d.kZero);
        assertConverges(new Pose2d(13.7, 4.7, Rotation2d.fromDegrees(25)), redTarget);
        assertConverges(new Pose2d(13.7, 3.5, Rotation2d.fromDegrees(-25)), redTarget);
    }

    @Test
    void correctsHeadingEvenWhenAlreadyCentered() {
        assertConverges(new Pose2d(TARGET.getTranslation(), Rotation2d.fromDegrees(90)), TARGET);
        assertConverges(new Pose2d(TARGET.getTranslation(), Rotation2d.fromDegrees(-179)), TARGET);
    }

    @Test
    void backsTowardHopperWithoutTurningAround() {
        HopperAlignmentController controller = new HopperAlignmentController(TARGET);
        Pose2d start = new Pose2d(TARGET.getX() - 0.6, TARGET.getY(), TARGET.getRotation());
        controller.reset(start);
        ChassisSpeeds output = controller.calculate(start, new ChassisSpeeds());
        assertTrue(output.vxMetersPerSecond < 0.0);
        assertEquals(0.0, output.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void reacquiresPositionIfVisionChangesDuringFinalTurn() {
        HopperAlignmentController controller = new HopperAlignmentController(TARGET);
        Pose2d centered = new Pose2d(TARGET.getTranslation(), Rotation2d.fromDegrees(140));
        controller.reset(centered);
        controller.calculate(centered, new ChassisSpeeds());
        runToGoal(controller, new Pose2d(TARGET.getX() - 0.25, TARGET.getY() + 0.3,
            centered.getRotation()), TARGET);
    }

    @Test
    void waitsForMotionToSettleAndCanBeReused() {
        HopperAlignmentController controller = new HopperAlignmentController(TARGET);
        controller.reset(TARGET);
        controller.calculate(TARGET, new ChassisSpeeds(0.1, 0.0, 0.0));
        assertFalse(controller.isFinished());
        controller.calculate(TARGET, new ChassisSpeeds(0.0, 0.0, 0.2));
        assertFalse(controller.isFinished());
        controller.calculate(TARGET, new ChassisSpeeds());
        assertTrue(controller.isFinished());
        Pose2d nextStart = new Pose2d(TARGET.getX() - 0.5, TARGET.getY() - 0.4, TARGET.getRotation());
        controller.reset(nextStart);
        assertFalse(controller.isFinished());
        runToGoal(controller, nextStart, TARGET);
    }

    private static void assertConverges(Pose2d start, Pose2d target) {
        HopperAlignmentController controller = new HopperAlignmentController(target);
        controller.reset(start);
        runToGoal(controller, start, target);
    }

    private static void runToGoal(HopperAlignmentController controller, Pose2d start, Pose2d target) {
        Pose2d pose = start;
        ChassisSpeeds measured = new ChassisSpeeds();
        // A differential drive with a small response lag: it can drive and turn, never strafe.
        // Large offsets may require a clearance maneuver longer than one autonomous attempt.
        // The command has a separate bounded timeout; here verify the steering converges.
        for (double elapsed = 0.0; elapsed < 30.0 && !controller.isFinished(); elapsed += DT) {
            ChassisSpeeds output = controller.calculate(pose, measured);
            assertEquals(0.0, output.vyMetersPerSecond);
            assertTrue(Math.abs(output.vxMetersPerSecond) <= kHopperAlignMaxSpeedMetersPerSecond);
            assertTrue(Math.abs(output.omegaRadiansPerSecond) <= kHopperAlignMaxTurnRateRadPerSec);
            measured = new ChassisSpeeds(
                measured.vxMetersPerSecond + 0.25 * (output.vxMetersPerSecond - measured.vxMetersPerSecond),
                0.0,
                measured.omegaRadiansPerSecond + 0.25 * (output.omegaRadiansPerSecond - measured.omegaRadiansPerSecond));
            pose = pose.exp(new Twist2d(measured.vxMetersPerSecond * DT, 0.0,
                measured.omegaRadiansPerSecond * DT));
        }
        assertTrue(controller.isFinished(), "Did not converge from " + start + "; ended at " + pose);
        assertTrue(pose.getTranslation().getDistance(target.getTranslation()) <= kHopperAlignToleranceMeters);
        assertTrue(Math.abs(MathUtil.angleModulus(target.getRotation().minus(pose.getRotation()).getRadians()))
            <= kHopperAlignHeadingToleranceRadians);
    }
}
