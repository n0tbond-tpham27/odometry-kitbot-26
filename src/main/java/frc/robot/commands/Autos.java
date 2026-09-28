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
     * feeding is done (registered as the "shoot" named command for PathPlanner autos).
     */
    public static Command shoot(Flywheel flywheel, Loader loader, IntakeClass intake) {
        double targetRPM = IOConstants.kFlywheelLaunchRPM;
        return Commands.deadline(
            Commands.sequence(
                Commands.waitUntil(() -> flywheel.isAtSpeed(targetRPM))
                    .withTimeout(AutoConstants.kShooterMaxSpinUpSeconds),
                // feed with the loader, running the intake alongside it
                Commands.deadline(
                    loader.runToFlywheelCommand().withTimeout(AutoConstants.kShooterFeedSeconds),
                    intake.runIntakeCommand())),
            flywheel.runShooterCommand(targetRPM)
        ).withName("autoShoot");
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
