// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.IOConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Flywheel extends SubsystemBase {

    private final TalonFX m_flywheelMotor; 
    private final TalonFX m_flywheelEncoder; // integrated encoder
    private final VoltageOut m_voltageRequest;
    private double m_currentFlywheelTargetRPM = kFlywheelDefaultTargetRPM;
    
    // Feedforward controller to run the shooter wheel in closed-loop, set the constants equal to
    // those calculated by SysId
    private final SimpleMotorFeedforward m_shooterFeedforward =
        new SimpleMotorFeedforward(
            IOConstants.kFlywheel_kS,
            IOConstants.kFlywheel_kV,
            IOConstants.kFlywheel_kA);

    // PID controller to run the shooter wheel in closed-loop, set the constants equal to those
    // calculated by SysId
    private final PIDController m_shooterFeedback =
        new PIDController(IOConstants.kFlywheel_kP, IOConstants.kFlywheel_kI, IOConstants.kFlywheel_kD);


    /** Creates a new Flywheel subsystem, configuring the shooter's TalonFX and closed-loop gains. */
    public Flywheel() {
        m_flywheelMotor = new TalonFX(kFlywheelMotorID);
        m_flywheelEncoder = m_flywheelMotor;
        m_voltageRequest = new VoltageOut(0.0);

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_flywheelMotor.getConfigurator().apply(flywheelConfig);
    }

    /**
     * Returns a command that runs the shooter at a specifc velocity.
     *
     * @param shooterSpeed The commanded shooter wheel speed in rotations per second
     */
    public Command runShooterCommand() {
        // Run shooter wheel at the current speed using a PID controller and feedforward.
        return run(() -> {
            m_flywheelMotor.setControl(m_voltageRequest.withOutput(
                m_shooterFeedback.calculate(m_flywheelEncoder.getVelocity().getValueAsDouble(), 
                                           m_currentFlywheelTargetRPM / 60.0)
                    + m_shooterFeedforward.calculate(m_currentFlywheelTargetRPM / 60.0)));
        })
        .finallyDo(
            () -> {
                m_flywheelMotor.stopMotor();
            })
        .withName("runShooter");
    }

    public Command stopShooterCommand() {
        return runOnce( () -> m_flywheelMotor.setVoltage(0.0));
    }


    public Command increaseShooterSpeedCommand() {
        return runOnce(() -> {
            if (m_currentFlywheelTargetRPM <= kFlywheelMaxRPM - 1000)
            m_currentFlywheelTargetRPM+=1000;}
        );
    }

    public Command decreaseShooterSpeedCommand() {
        return runOnce(() -> {
                if (m_currentFlywheelTargetRPM >= 1000) 
                    m_currentFlywheelTargetRPM-=1000;
            }
        );
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        // 1. Extract the raw numeric values from Phoenix 6 StatusSignals
        double actualVelocityRPS = m_flywheelMotor.getVelocity().getValueAsDouble();
        double actualPositionRotations = m_flywheelMotor.getPosition().getValueAsDouble();
        double motorCurrentAmps = m_flywheelMotor.getStatorCurrent().getValueAsDouble();

        // 2. Push fields to NetworkTables for SmartDashboard / AdvantageScope
        SmartDashboard.putNumber("Flywheel/Actual Velocity (RPS)", actualVelocityRPS);
        SmartDashboard.putNumber("Flywheel/Target Velocity (RPS)", m_currentFlywheelTargetRPM/60.0);
        SmartDashboard.putNumber("Flywheel/Actual Velocity (RPM)", actualVelocityRPS * 60.0);
        SmartDashboard.putNumber("Flywheel/Target Velocity (RPM)", m_currentFlywheelTargetRPM);
        SmartDashboard.putNumber("Flywheel/Position (Rotations)", actualPositionRotations);
        SmartDashboard.putNumber("Flywheel/Stator Current (A)", motorCurrentAmps);
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    // ======= Logging Methods ========
    @Logged
    public double getShooterSpeedRPS() {
        return m_flywheelMotor.getVelocity().getValueAsDouble();
    }

    @Logged
    public double getShooterSpeedRPM() {
        return m_flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }

}
