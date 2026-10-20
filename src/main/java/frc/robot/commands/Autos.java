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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
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
     * For use at Hopper Center (back to the hopper, where the Limelight sees its tags): snaps the
     * pose to the averaged vision position, then turns and drives onto Hopper Center and restores
     * its final heading. Corrects left/right and forward/back starting offsets as well as path drift.
     * Registered as the "visionAlign" named command. Without fresh tags it holds still and skips alignment.
     */
    public static Command visionAlign(DriveSubsystem drive) {
        return Commands.sequence(
            drive.resetPoseToVisionCommand(),
            drive.alignToHopperCenterCommand()
        ).withName("visionAlign");
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
