// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeClass;
import frc.robot.subsystems.Loader;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Autos {
    private static final Set<String> HOPPER_CENTER_AUTOS = Set.of(
        "Center Left", "Center Left 2x", "Center Left 3x",
        "Center Right", "Center Right 2x", "Center Right 3x");

    public static boolean startsAtHopperCenter(String autoName) {
        return HOPPER_CENTER_AUTOS.contains(autoName);
    }

    /** Trust the team's centered, square placement; leave subsequent vision corrections to the return. */
    public static Command hopperStart(HopperTagAlignmentCommand.IO drive, Subsystem requirement,
            Supplier<Pose2d> startingPose) {
        return Commands.runOnce(() -> {
            drive.stopMotors();
            Pose2d pose = startingPose.get();
            // resetOdometry records the current raw gyro as the baseline for this field angle.
            drive.resetOdometry(pose);
            SmartDashboard.putString("Auto/Start Pose Source", "known centered hopper placement");
            SmartDashboard.putNumber("Auto/Start X (m)", pose.getX());
            SmartDashboard.putNumber("Auto/Start Y (m)", pose.getY());
            SmartDashboard.putNumber("Auto/Start Heading (deg)", pose.getRotation().getDegrees());
        }, requirement).withName("hopperStart");
    }

    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(ExampleSubsystem subsystem) {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    /** Drive a fixed distance
     * 
     */
    public static Command driveDistance(DriveSubsystem driveSubsystem) {
        return Commands.sequence(
            driveSubsystem.resetEncoderCommand(),
            driveSubsystem.arcadeDriveCommand(()->0.4, ()->0.0)
                .until(() -> driveSubsystem.getAverageDistanceMeters() > AutoConstants.kDistanceTargetMeters)
                .finallyDo(() -> driveSubsystem.stopMotors())
        );
    }

    /**
     * Spins the flywheel up to the launch speed, then feeds it with the loader once it's up to
     * speed (or after kShooterMaxSpinUpSeconds, whichever comes first). Ends on its own once
     * feeding is done, with a tiny forward-first drivetrain wiggle while feeding
     * (registered as the "shoot" named command for PathPlanner autos).
     */
    public static Command shoot(DriveSubsystem drive, Flywheel flywheel, Loader loader, IntakeClass intake) {
        double targetRPM = IOConstants.kFlywheelLaunchRPM;
        return Commands.deadline(
            Commands.sequence(
                drive.stopDriveCommand(),
                Commands.waitUntil(() -> flywheel.isAtSpeed(targetRPM))
                    .withTimeout(AutoConstants.kShooterMaxSpinUpSeconds),
                // feed with the loader, running the intake alongside it
                Commands.deadline(
                    loader.runToFlywheelCommand().withTimeout(AutoConstants.kShooterFeedSeconds),
                    intake.runIntakeCommand(),
                    drive.shootingWiggleCommand())),
            flywheel.runShooterCommand(targetRPM)
        ).withName("autoShoot");
    }

    /**
     * Attempts to center on the hopper tag. Missing vision or an unsuccessful correction must
     * finish this step so the autonomous routine can continue using its existing odometry.
     */
    public static Command visionAlign(DriveSubsystem drive) {
        return boundedVisionAlign(drive.alignToHopperReturnCommand(
                AutoConstants.kHopperApproachMaxSpeedMetersPerSecond), drive::stopMotors,
                AutoConstants.kHopperReturnAlignTimeoutSeconds)
            .andThen(Commands.runOnce(() -> {
                boolean aligned = drive.isHopperTagAligned();
                SmartDashboard.putBoolean("Auto/Vision Correction Applied", aligned);
                SmartDashboard.putString("Auto/Vision Align Status", aligned ? "aligned"
                    : "continuing with odometry: " + SmartDashboard.getString("Vision/Align Status", "vision unavailable"));
            }))
            .withName("visionAlign");
    }

    /** The auto always gets its drivetrain back, even if an alignment command never finishes. */
    static Command boundedVisionAlign(Command attempt, Runnable stopDrive) {
        return boundedVisionAlign(attempt, stopDrive, AutoConstants.kHopperAlignTimeoutSeconds);
    }

    static Command boundedVisionAlign(Command attempt, Runnable stopDrive, double timeoutSeconds) {
        return attempt.withTimeout(timeoutSeconds).finallyDo(stopDrive);
    }

    /** Correct lateral drift during the return, using a slow straight path only if vision never acquires. */
    public static Command resetBack(DriveSubsystem drive, String fallbackPathName) {
        return Commands.defer(() -> {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(fallbackPathName);
                Command fallback = Commands.runOnce(() -> SmartDashboard.putString(
                    "Auto/Return Mode", "slow odometry fallback: no usable vision"))
                    .andThen(AutoBuilder.followPath(path)).finallyDo(drive::stopMotors);
                return Commands.runOnce(() -> SmartDashboard.putString("Auto/Return Mode", "vision-guided reverse"))
                    .andThen(boundedVisionAlign(drive.alignToHopperReturnCommand(
                        AutoConstants.kHopperApproachMaxSpeedMetersPerSecond), drive::stopMotors,
                        AutoConstants.kHopperReturnAlignTimeoutSeconds))
                    .andThen(returnAfterVision(fallback, drive::isHopperTagAligned,
                        drive::wasHopperPoseAcquired, drive::stopMotors));
            } catch (Exception exception) {
                return drive.runOnce(() -> {
                    drive.stopMotors();
                    DriverStation.reportError("Cannot load hopper return: " + exception.getMessage(), false);
                });
            }
        }, Set.of(drive))
            .beforeStarting(() -> drive.setHopperReturnActive(true))
            .finallyDo(() -> {
                drive.stopMotors();
                drive.setHopperReturnActive(false);
            }).withName("Vision Guided Reset Back");
    }

    /** Never start a blind return after a partial vision-guided maneuver. */
    static Command returnAfterVision(Command fallback, BooleanSupplier aligned,
            BooleanSupplier acquired, Runnable stopDrive) {
        return Commands.either(
            Commands.runOnce(() -> SmartDashboard.putString("Auto/Return Mode", "vision confirmed")),
            Commands.either(
                Commands.runOnce(() -> {
                    stopDrive.run();
                    SmartDashboard.putString("Auto/Return Mode", "correction incomplete; stopped");
                }), fallback, acquired),
            aligned);
    }

    /**
     * Runs the intake and loader together until interrupted. Used by the "intake" event marker,
     * where the marker's zone decides how long it runs.
     */
    public static Command intake(IntakeClass intake, Loader loader) {
        return Commands.parallel(intake.runIntakeCommand(), loader.runWithIntakeCommand())
            .withName("autoIntake");
    }

    /**
     * Same as {@link #intake}, but ends on its own after kIntakeTimeoutSeconds so it can be used
     * as a step in a sequence (registered as the "intake" named command for PathPlanner autos).
     */
    public static Command intakeWithTimeout(IntakeClass intake, Loader loader) {
        return intake(intake, loader)
            .withTimeout(AutoConstants.kIntakeTimeoutSeconds)
            .withName("autoIntakeTimed");
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
