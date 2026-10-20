package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Small forward-first shooting pulses, measured from one fixed encoder baseline. */
public final class ShootingWiggleController {
    /** Heading error is start heading minus current heading, in radians. */
    public ChassisSpeeds calculate(double elapsedSeconds, double forwardMeters, double headingErrorRadians) {
        long interval = (long) Math.floor(elapsedSeconds / kShootWiggleIntervalSeconds);
        double timeInInterval = elapsedSeconds - interval * kShootWiggleIntervalSeconds;
        if (timeInInterval >= kShootWiggleMoveSeconds) {
            return new ChassisSpeeds();
        }

        boolean forward = interval % 2 == 0;
        double target = forward ? kShootWiggleForwardMeters : kShootWiggleReturnMeters;
        double speed = kShootWigglePositionP * (target - forwardMeters);
        // Only move in the current pulse's direction. In particular, if a forward pulse is
        // blocked, the next pulse must not back into the hopper to make up for lost movement.
        speed = forward
            ? MathUtil.clamp(speed, 0.0, kShootWiggleMaxSpeedMetersPerSecond)
            : MathUtil.clamp(speed, -kShootWiggleMaxSpeedMetersPerSecond, 0.0);

        // Hold the shooting heading without making either wheel oppose the pulse's direction.
        double maxTurnRate = Math.min(kShootWiggleMaxTurnRateRadPerSec,
            2.0 * Math.abs(speed) / kDriveTrackWidthMeters);
        double turnRate = MathUtil.clamp(
            kShootWiggleHeadingP * MathUtil.angleModulus(headingErrorRadians), -maxTurnRate, maxTurnRate);
        return new ChassisSpeeds(speed, 0.0, turnRate);
    }
}
