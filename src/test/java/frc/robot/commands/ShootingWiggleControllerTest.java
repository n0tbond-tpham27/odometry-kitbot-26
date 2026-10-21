package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ShootingWiggleControllerTest {
    private final ShootingWiggleController controller = new ShootingWiggleController();

    @Test
    void firstPulseIsForwardEvenAfterPreviousShots() {
        assertTrue(controller.calculate(0.0, 0.0, 0.0).vxMetersPerSecond > 0.0);
        controller.calculate(kShootWiggleIntervalSeconds + 0.01, kShootWiggleForwardMeters, 0.0);
        assertTrue(controller.calculate(0.0, 0.0, 0.0).vxMetersPerSecond > 0.0);
    }

    @Test
    void blockedForwardPulseNeverResultsInReverseIntoHopper() {
        for (double distance : new double[] {-0.005, 0.0, kShootWiggleReturnMeters}) {
            ChassisSpeeds output = controller.calculate(kShootWiggleIntervalSeconds + 0.01, distance, 0.1);
            assertEquals(0.0, output.vxMetersPerSecond);
            assertEquals(0.0, output.omegaRadiansPerSecond);
        }
    }

    @Test
    void reversesAfterMovingForwardAndRestsBetweenPulses() {
        assertTrue(controller.calculate(kShootWiggleIntervalSeconds + 0.01,
            kShootWiggleForwardMeters, 0.0).vxMetersPerSecond < 0.0);
        for (int interval = 0; interval < 4; interval++) {
            ChassisSpeeds output = controller.calculate(
                interval * kShootWiggleIntervalSeconds + kShootWiggleMoveSeconds + 0.01, 0.015, 0.2);
            assertEquals(0.0, output.vxMetersPerSecond);
            assertEquals(0.0, output.omegaRadiansPerSecond);
        }
    }

    @Test
    void repeatedPulsesRemainTinyWithDrivetrainResponseLag() {
        double position = 0.0;
        double velocity = 0.0;
        boolean reversed = false;
        for (int step = 0; step < 1500; step++) {
            ChassisSpeeds output = controller.calculate(step * 0.02, position, 0.0);
            assertTrue(Math.abs(output.vxMetersPerSecond) <= kShootWiggleMaxSpeedMetersPerSecond);
            reversed |= output.vxMetersPerSecond < 0.0;
            velocity += 0.25 * (output.vxMetersPerSecond - velocity);
            position += velocity * 0.02;
            assertTrue(position >= 0.0, "Backed behind shooting start");
            assertTrue(position <= kShootWiggleForwardMeters + 0.002, "Accumulated forward drift");
        }
        assertTrue(reversed, "Must actually alternate rather than only creep forward");
    }

    @Test
    void headingHoldDoesNotReverseEitherWheelDuringForwardPulse() {
        for (double headingError : new double[] {-3.0, -0.1, 0.1, 3.0}) {
            ChassisSpeeds output = controller.calculate(0.0, 0.0, headingError);
            double wheelDifference = output.omegaRadiansPerSecond * kDriveTrackWidthMeters / 2.0;
            assertTrue(output.vxMetersPerSecond - wheelDifference >= -1e-9);
            assertTrue(output.vxMetersPerSecond + wheelDifference >= -1e-9);
        }
    }
}
