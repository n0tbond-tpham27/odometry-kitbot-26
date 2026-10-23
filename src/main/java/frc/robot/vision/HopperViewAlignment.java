package frc.robot.vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.HopperStartView.Reference;
import frc.robot.vision.Limelight.TargetMeasurement;

/** Both tag bearings determine translation; the gyro independently determines rotation. */
public final class HopperViewAlignment {
    public record View(double mainX, double companionX, double mainY, double companionY) {
        public boolean matches(TargetMeasurement sample, double toleranceDegrees) {
            return Math.abs(sample.imageHorizontalDegrees() - mainX) <= toleranceDegrees
                && Math.abs(sample.companionImageHorizontalDegrees() - companionX) <= toleranceDegrees
                && verticalMatches(sample.imageVerticalDegrees(), mainY, toleranceDegrees)
                && verticalMatches(sample.companionImageVerticalDegrees(), companionY, toleranceDegrees);
        }

        private static boolean verticalMatches(double measured, double desired, double tolerance) {
            // Legacy captures can contain only horizontal bearings. The LL4 supplies both.
            return !Double.isFinite(desired) || (Double.isFinite(measured) && Math.abs(measured - desired) <= tolerance);
        }
    }

    /**
     * Approximate centers measured from the supplied photo, excluding its browser border.
     * Native image viewport: 1184 x 740 in the screenshot, matching LL4's 1280 x 800 ratio.
     * Refine these factory-lens estimates with a stationary live capture in this position.
     */
    public static View photo() {
        return new View(horizontal(361.0 / 1184.0), horizontal(132.0 / 1184.0),
            vertical(147.0 / 740.0), vertical(152.0 / 740.0));
    }

    public static View from(Reference reference) {
        return new View(reference.imageHorizontalDegrees(), reference.companionImageHorizontalDegrees(),
            reference.imageVerticalDegrees(), reference.companionImageVerticalDegrees());
    }

    private static double horizontal(double normalizedX) {
        return Math.toDegrees(Math.atan((2.0 * normalizedX - 1.0)
            * Math.tan(Math.toRadians(kHubPhotoHorizontalFovDegrees / 2.0))));
    }

    private static double vertical(double normalizedY) {
        return Math.toDegrees(Math.atan((1.0 - 2.0 * normalizedY)
            * Math.tan(Math.toRadians(kHubPhotoVerticalFovDegrees / 2.0))));
    }

    /**
     * Local visual correction near the hub. Remove gyro yaw before comparing bearings,
     * then use the pair's tangent separation for range and its common offset for lateral
     * position. Turning to put one tag at the crosshair cannot satisfy this observation.
     */
    public static Optional<Pose2d> estimatePose(TargetMeasurement sample, View target,
            Pose2d desiredPose, Rotation2d gyroHeading) {
        double yaw = MathUtil.angleModulus(gyroHeading.minus(desiredPose.getRotation()).getRadians());
        double distance = sample.cameraOutwardMeters();
        if (!sample.hasHopperPair() || !Double.isFinite(distance) || distance <= 0.1
                || !Double.isFinite(yaw) || Math.abs(yaw) > Math.toRadians(35.0)) {
            return Optional.empty();
        }
        double main = Math.tan(Math.toRadians(sample.imageHorizontalDegrees()) - yaw);
        double companion = Math.tan(Math.toRadians(sample.companionImageHorizontalDegrees()) - yaw);
        double targetMain = Math.tan(Math.toRadians(target.mainX()));
        double targetCompanion = Math.tan(Math.toRadians(target.companionX()));
        double span = main - companion;
        double targetSpan = targetMain - targetCompanion;
        double ratio = span / targetSpan;
        if (!Double.isFinite(ratio) || span <= 0.02 || targetSpan <= 0.02 || ratio < 0.15 || ratio > 2.0) {
            return Optional.empty();
        }
        double desiredDistance = distance * ratio;
        double outwardError = distance - desiredDistance;
        double rightError = -distance * main + desiredDistance * targetMain;
        // An off-center camera moves around the gyro's rotation center during a turn.
        // Remove that motion rather than treating it as sideways chassis movement.
        Translation2d mountNow = new Translation2d(sample.cameraOutwardMeters() - sample.outwardMeters(),
            sample.cameraRightMeters() - sample.rightMeters());
        Translation2d mountAtGoal = mountNow.rotateBy(Rotation2d.fromRadians(-yaw));
        outwardError -= mountNow.getX() - mountAtGoal.getX();
        rightError -= mountNow.getY() - mountAtGoal.getY();
        if (Math.abs(outwardError) > 2.0 || Math.abs(rightError) > 1.0) {
            return Optional.empty();
        }
        Translation2d error = new Translation2d(outwardError, rightError).rotateBy(desiredPose.getRotation());
        return Optional.of(new Pose2d(desiredPose.getTranslation().plus(error), gyroHeading));
    }

    private HopperViewAlignment() {}
}
