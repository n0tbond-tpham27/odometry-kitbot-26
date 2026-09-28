// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeClass;
import frc.robot.subsystems.Loader;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final Flywheel m_flywheel = new Flywheel();
    private final IntakeClass m_intake = new IntakeClass();
    private final Loader m_loader = new Loader();
  

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // Dashboard dropdown for picking the autonomous routine
    @NotLogged
    private final SendableChooser<Command> m_autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Named commands must be registered before any PathPlanner auto is loaded
        NamedCommands.registerCommand("shoot", Autos.shoot(m_flywheel, m_loader, m_intake));
        // "intake" runs until cancelled: an event marker zone in a path starts it when the robot
        // enters the zone and cancels it at the end of the zone.
        NamedCommands.registerCommand("intake", Autos.intake(m_intake, m_loader));
        // Same thing but ending on its own, for use as a step inside an auto's sequence.
        NamedCommands.registerCommand("intakeTimed", Autos.intakeWithTimeout(m_intake, m_loader));

        // Lists every auto in deploy/pathplanner/autos, plus our non-PathPlanner routine
        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.addOption("Drive Distance (no path)", Autos.driveDistance(m_driveSubsystem));
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        // Configure the trigger bindings
        configureBindings();
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
    private void configureBindings() {

        // Control the drive with split-stick arcade controls (left stick Y drives, right stick X turns)
        m_driveSubsystem.setDefaultCommand(
            m_driveSubsystem.arcadeDriveCommand(
                () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OperatorConstants.kDriveDeadband)));

        // Left trigger SHOOTS: spin up, then feed with the loader once up to speed
        m_driverController.leftTrigger(OperatorConstants.kTriggerThreshold)
            .whileTrue(shootCommand(IOConstants.kFlywheelLaunchRPM));

        // Right trigger INTAKES: intake + loader, no flywheel
        m_driverController.rightTrigger(OperatorConstants.kTriggerThreshold)
            .whileTrue(Commands.parallel(m_intake.runIntakeCommand(), m_loader.runWithIntakeCommand()));

        // Left bumper EJECTS back out through the intake
        m_driverController.leftBumper()
            .whileTrue(Commands.parallel(m_intake.reverseIntakeCommand(), m_loader.runWithIntakeCommand(true)));

        // X toggles pre-spinning the flywheel so shots start faster
        m_driverController.x().toggleOnTrue(m_flywheel.runShooterCommand(IOConstants.kFlywheelSpinUpRPM));

        // Y backs a stuck game piece out of the loader (loader only, run in reverse)
        m_driverController.y().whileTrue(m_loader.runWithIntakeCommand(true));

        // Range shots: A = high, B and right bumper = long range
        m_driverController.a().whileTrue(shootCommand(IOConstants.kFlywheelHighShotRPM));
        m_driverController.b().whileTrue(shootCommand(IOConstants.kFlywheelUltraShotRPM));
        m_driverController.rightBumper().whileTrue(shootCommand(IOConstants.kFlywheelUltraShotRPM));

        // Put the robot back at the starting pose (line it up on the field first)
        m_driverController.start().onTrue(m_driveSubsystem.resetOdometryCommand());

        // Zero the drive encoders — used when calibrating distance against a tape measure
        m_driverController.back().onTrue(m_driveSubsystem.resetEncoderCommand());
    }

    /**
     * Runs the flywheel at {@code targetRPM} and, once it is up to speed, feeds it with the loader
     * and intake together (the intake keeps game pieces moving up behind the one being shot).
     * Runs until the button is released.
     */
    private Command shootCommand(double targetRPM) {
        return Commands.parallel(
            m_flywheel.runShooterCommand(targetRPM),
            Commands.waitUntil(() -> m_flywheel.isAtSpeed(targetRPM))
                .andThen(Commands.parallel(
                    m_loader.runToFlywheelCommand(),
                    m_intake.runIntakeCommand())));
    }

    /**
     * Shows the selected auto's starting pose on the Field view (as "Auto Start"), flipped to the
     * red side when on red. Called while disabled, so the drive team can line the robot up with it.
     */
    public void updateAutoStartPreview() {
        Pose2d start = null;
        if (m_autoChooser.getSelected() instanceof PathPlannerAuto auto && auto.getStartingPose() != null) {
            start = AutoBuilder.shouldFlip()
                ? FlippingUtil.flipFieldPose(auto.getStartingPose())
                : auto.getStartingPose();
        }
        m_driveSubsystem.showAutoStartPose(start);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
    
}
