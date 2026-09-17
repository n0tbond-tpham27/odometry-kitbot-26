// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IOConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
  
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Reference implementation of the intake mechanism (1x CTRE Kraken X60 / integrated TalonFX).
 *
 * <p>Runs open-loop (duty cycle) rather than closed-loop velocity control: intake surface speed
 * doesn't need to be precise the way the Flywheel's does, so a simple percent-output command is
 * sufficient and much simpler to read/annotate first.
 *
 * <p>Note: {@code kIntakeDefaultTargetRPM} in {@link frc.robot.Constants.IOConstants} is not used
 * here on purpose. Wiring the intake up to closed-loop RPM control using that constant (the same
 * pattern as {@link Flywheel}) is a natural extension task once open-loop is working.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class IntakeClass extends SubsystemBase {

    private final TalonFX m_intakeMotor;
    private final DutyCycleOut m_dutyCycleRequest;

    /** Creates a new IntakeClass. */
    public IntakeClass() {
        m_intakeMotor = new TalonFX(kIntakeMotorID);
        m_dutyCycleRequest = new DutyCycleOut(0.0);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // or CounterClockwise_Positive
        intakeConfig.CurrentLimits.SupplyCurrentLimit = kIntakeMotorCurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_intakeMotor.getConfigurator().apply(intakeConfig);
    }

    /**
     * Runs the intake forward (pulling a game piece in) at a fixed open-loop output while the
     * command is active. Stops the motor automatically when the command ends or is interrupted.
     */
    public Command runIntakeCommand() {
        return run(() -> m_intakeMotor.setControl(m_dutyCycleRequest.withOutput(kIntakeDefaultOutput)))
            .finallyDo(() -> m_intakeMotor.stopMotor())
            .withName("runIntake");
    }

    /**
     * Runs the intake in reverse (e.g. to clear a jam or eject a game piece).
     */
    public Command reverseIntakeCommand() {
        return run(() -> m_intakeMotor.setControl(m_dutyCycleRequest.withOutput(-kIntakeDefaultOutput)))
            .finallyDo(() -> m_intakeMotor.stopMotor())
            .withName("reverseIntake");
    }

    /** Immediately stops the intake motor. */
    public Command stopIntakeCommand() {
        return runOnce(() -> m_intakeMotor.stopMotor());
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor
     * such as a beam break to detect a held game piece).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Velocity (RPM)", getIntakeSpeedRPM());
        SmartDashboard.putNumber("Intake/Stator Current (A)", m_intakeMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    // ======= Logging Methods ========
    @Logged
    public double getIntakeSpeedRPM() {
        return m_intakeMotor.getVelocity().getValueAsDouble() * 60.0;
    }
}
