// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AutoConstants.kLTVDtSeconds;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OdometryConstants.*;
import static frc.robot.Constants.RobotConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class DriveSubsystem extends SubsystemBase {
    // --- motors ---
    private final SparkMax m_leftLeader;
    private final SparkMax m_leftFollower;
    private final SparkMax m_rightLeader;
    private final SparkMax m_rightFollower;

    // --- Differential Drive subsystem 
    private final DifferentialDrive m_differentialDrive;

    // --- Encoders ---
    private final CANcoder m_rightEncoder;

    // --- Gryo ----
    private final Pigeon2 m_pigeon2 = new Pigeon2(kPigeon2ID);  
    

    // --- Odometry ---
    private final DifferentialDriveKinematics m_kinematics =
        new DifferentialDriveKinematics(kDriveTrackWidthMeters);
    private final DifferentialDriveOdometry m_odometry;
    private double m_virtualLeftEncoderReading; // use to simulate left encoder
    /** Gyro heading (radians) at the last encoder reset; the virtual left encoder is measured from here. */
    private double m_headingAtResetRadians = 0.0;
    /** Raw encoder reading at the last reset; subtracted out so distances start from zero. */
    private double m_rightEncoderOffsetRotations = 0.0;


    private final Field2d m_field = new Field2d(); // field object for display

    // --- Measuring (compare odometry against a tape measure) ---
    /** Odometry pose when the current measurement was started. */
    private Pose2d m_markPose = Pose2d.kZero;
    /** Continuous gyro heading (degrees) when the current measurement was started. */
    private double m_markGyroDegrees = 0.0;
    /** Distance the robot's estimated position has travelled along its route since the mark. */
    private double m_pathLengthSinceMarkMeters = 0.0;
    /** Pose from the previous loop, for accumulating path length. */
    private Pose2d m_previousPose = Pose2d.kZero;
    /** Right wheel distance when the current measurement was started. */
    private double m_markRightDistanceMeters = 0.0;

    /** Set whenever something commands the drive motors; checked and cleared in periodic(). */
    private boolean m_drivenThisLoop = false;
    // Last voltage commanded to each side (forward-positive). The simulated SparkMaxes don't
    // report their output back, so simulationPeriodic() reads these instead.
    private double m_leftCommandedVolts = 0.0;
    private double m_rightCommandedVolts = 0.0;

    // --- Simulation ---
    /** Physics model of the drivetrain; only used in simulation. */
    private final DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDualCIMPerSide, KitbotGearing.k8p45, KitbotWheelSize.kSixInch, null);

    /** Creates a new DriveSubsystem, configuring the KitBot's differential drivetrain motors and right-side CANcoder. */
    public DriveSubsystem() {

        // Create brushed motors for a KitBot-style CIM drivetrain
        m_leftLeader = new SparkMax(kLeftLeaderId, MotorType.kBrushed);
        m_leftFollower = new SparkMax(kLeftFollowerId, MotorType.kBrushed);
        m_rightLeader = new SparkMax(kRightLeaderId, MotorType.kBrushed);
        m_rightFollower = new SparkMax(kRightFollowerId, MotorType.kBrushed);


        // Left leader: invert so that positive values drive both sides forward
        SparkMaxConfig m_leftLeaderConfig = new SparkMaxConfig();
        m_leftLeaderConfig.voltageCompensation(12);
        m_leftLeaderConfig.smartCurrentLimit(kDriveMotorCurrentLimit);
        m_leftLeaderConfig.inverted(kLeftLeaderReversed);
        m_leftLeader.configure(m_leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right leader: not inverted
        SparkMaxConfig m_rightLeaderConfig = new SparkMaxConfig();
        m_rightLeaderConfig.voltageCompensation(12);
        m_rightLeaderConfig.smartCurrentLimit(kDriveMotorCurrentLimit);
        m_rightLeaderConfig.inverted(kRightLeaderReversed);
        m_rightLeader.configure(m_rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Followers mirror their respective leaders
        SparkMaxConfig m_leftFollowerConfig = new SparkMaxConfig();
        m_leftFollowerConfig.follow(m_leftLeader);
        m_leftFollower.configure(m_leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig m_rightFollowerConfig = new SparkMaxConfig();
        m_rightFollowerConfig.follow(m_rightLeader);
        m_rightFollower.configure(m_rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Built-in encoders
        m_rightEncoder = new CANcoder(kRightEncoderID);
        m_rightEncoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withSensorDirection(kRightEncoderReversed
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive));

        // Odometry initialization. Wait for the first real reading from each device before using
        // it as a baseline: until a status frame arrives these report 0, and adopting that as the
        // baseline makes the pose jump as soon as the true value shows up.
        m_rightEncoder.getPosition().waitForUpdate(0.25);
        m_pigeon2.getYaw().waitForUpdate(0.25);

        // Zeroing is done in software (an offset) rather than by setPosition()/setYaw() on the
        // devices, so a reset takes effect on the very next loop instead of a few ms later.
        m_rightEncoderOffsetRotations = m_rightEncoder.getPosition().getValueAsDouble();
        m_headingAtResetRadians = getHeadingRotation2d().getRadians();
        m_virtualLeftEncoderReading = 0.0;
        // Start odometry at the configured starting pose ((0,0) is the bottom-left corner of the
        // field), with both wheel distances measured from the offsets just taken.
        m_odometry = new DifferentialDriveOdometry(
            getHeadingRotation2d(), 0.0, 0.0, getStartingPose());

        // initialize differential drive object
        m_differentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        SmartDashboard.putData("Field", m_field);

        // Compass dials for Elastic/Shuffleboard's Gyro widget, updated every loop by our own code
        // (a generic LiveWindow gyro widget only updates in Test mode, so it looks frozen otherwise).
        // Both are CCW-positive: set the widget to counter-clockwise positive so the dial turns the
        // same way as the robot.
        SmartDashboard.putData("Drive/Gyro", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", this::getHeadingDegrees, null);
        });
        SmartDashboard.putData("Drive/Odometry Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", () -> getPose().getRotation().getDegrees(), null);
        });

        // Dashboard button that starts a new measurement (see markMeasurement)
        SmartDashboard.putData("Measure/Mark Here", runOnce(this::markMeasurement)
            .ignoringDisable(true).withName("Mark Here"));
        markMeasurement();

        // Dashboard button: spin in place and compute the effective track width from the gyro
        SmartDashboard.putData("Measure/Find Track Width", findTrackWidthCommand());

        configurePathPlanner();
    }

    /** Hooks this drivetrain into PathPlanner's AutoBuilder, following paths with the LTV controller. */
    private void configurePathPlanner() {
        RobotConfig config;
        try {
            // Reads the Robot Config set in the PathPlanner GUI (deploy/pathplanner/settings.json)
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner robot config: " + e.getMessage(), e.getStackTrace());
            return;
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPLTVController(kLTVDtSeconds),
            config,
            // Paths are drawn for the blue side; mirror them when we're on red
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
    }

    // ----------- Driving Commands --------------

    /**
     * A split-stick arcade command, with forward/backward controlled by the left hand, 
     * and turning controlled by right; values should be negated when sent from controller
     * due to differences in coordinate orientation. 
     * @param fwd 
     * @param rot
     * @return
     */
    public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
        return run(() -> {
            m_differentialDrive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble());
            // arcadeDrive sets duty cycles; with 12 V voltage compensation, volts = duty cycle * 12
            m_leftCommandedVolts = m_leftLeader.get() * 12.0;
            m_rightCommandedVolts = m_rightLeader.get() * 12.0;
            m_drivenThisLoop = true;
        })
            .withName("arcadeDrive");
    }

    public Command stopDriveCommand() {
        return runOnce( () -> this.stopMotors() );
    }

    public Command resetEncoderCommand() {
        return runOnce( () -> this.resetEncoders() );
    }

    /** Resets odometry so the robot is at the configured starting pose. */
    public Command resetOdometryCommand() {
        return runOnce( () -> this.resetOdometry(getStartingPose()) )
            .ignoringDisable(true)
            .withName("resetOdometry");
    }

     /** 
     * Stops drivetrain motors immediately
     */
    public void stopMotors() {
        m_differentialDrive.stopMotor(); // also feeds the Motor Safety watchdog
        m_leftCommandedVolts = 0.0;
        m_rightCommandedVolts = 0.0;
        m_drivenThisLoop = true;
    }

    /**
     * Applies voltages directly to each side (bypassing DifferentialDrive) and feeds the Motor
     * Safety watchdog so it doesn't disable our motors.
     */
    private void setMotorVoltages(double leftVolts, double rightVolts) {
        m_leftLeader.setVoltage(leftVolts);
        m_rightLeader.setVoltage(rightVolts);
        m_leftCommandedVolts = leftVolts;
        m_rightCommandedVolts = rightVolts;
        m_differentialDrive.feed();
        m_drivenThisLoop = true;
    }


    // --------- Odometry and Helper Functions -----------

    /** Returns the robot's estimated pose on the field (meters, CCW-positive heading). */
    @Logged
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Shows where the selected auto expects the robot to start, as a second robot ("Auto Start")
     * on the Field view, and as numbers under Auto/. At auto start the odometry is simply told it
     * is at this pose, so the real robot has to be physically placed there, facing this way.
     * Pass null to clear it (e.g. no PathPlanner auto selected).
     */
    public void showAutoStartPose(Pose2d pose) {
        if (pose == null) {
            m_field.getObject("Auto Start").setPoses();
            return;
        }
        m_field.getObject("Auto Start").setPose(pose);
        SmartDashboard.putNumber("Auto/Start X (m)", pose.getX());
        SmartDashboard.putNumber("Auto/Start Y (m)", pose.getY());
        SmartDashboard.putNumber("Auto/Start Heading (deg)", pose.getRotation().getDegrees());
    }

    /** Returns the starting pose configured in {@code OdometryConstants}. */
    public static Pose2d getStartingPose() {
        return new Pose2d(kStartingXMeters, kStartingYMeters,
            Rotation2d.fromDegrees(kStartingHeadingDegrees));
    }

    /**
     * Resets odometry so the robot is at {@code pose}. Encoders and gyro are left alone;
     * the odometry object records their current readings as the new baseline.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            getHeadingRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
        markMeasurement(); // the pose just jumped, so start measuring from the new one
    }

    /**
     * Starts a new measurement from where the robot is now: the Measure/ values on the dashboard
     * then show how far and in what direction odometry thinks the robot has moved, and how much
     * it has turned, since this moment. Doesn't change odometry itself.
     */
    public void markMeasurement() {
        m_markPose = getPose();
        m_previousPose = m_markPose;
        m_markGyroDegrees = getHeadingDegrees();
        m_markRightDistanceMeters = getRightDistanceMeters();
        m_pathLengthSinceMarkMeters = 0.0;
    }

    /**
     * Measures the effective track width using the gyro: spins the robot slowly in place for two
     * full turns, then divides the distance the right wheel travelled by the angle the gyro saw.
     * In an in-place spin each side travels trackWidth * angle / 2, so
     * trackWidth = 2 * wheelDistance / angle. The result is published as
     * "Measure/Effective Track Width (m)"; copy it into kDriveTrackWidthMeters.
     *
     * <p>Calibrate kEncoderDistanceCalibration first (tape-measure push test): this result is
     * only as accurate as the wheel distance it's computed from.
     */
    public Command findTrackWidthCommand() {
        return runOnce(this::markMeasurement)
            .andThen(run(() -> setMotorVoltages(-kTrackWidthSpinVolts, kTrackWidthSpinVolts))
                .until(() -> Math.abs(getHeadingDegrees() - m_markGyroDegrees) >= 720.0))
            .withTimeout(20.0)
            .finallyDo(interrupted -> {
                stopMotors();
                double turnedRadians = Math.toRadians(Math.abs(getHeadingDegrees() - m_markGyroDegrees));
                double wheelMeters = Math.abs(getRightDistanceMeters() - m_markRightDistanceMeters);
                if (turnedRadians > Math.toRadians(90.0)) {
                    SmartDashboard.putNumber("Measure/Effective Track Width (m)",
                        2.0 * wheelMeters / turnedRadians);
                }
            })
            .withName("Find Track Width");
    }

    /**
     * Returns encoder rotations since the last reset, with no distance conversion applied.
     * Used to calibrate {@code kEncoderDistanceCalibration}: reset, push the robot a measured
     * distance, then divide that distance by this reading to get true meters per rotation.
     */
    @Logged
    public double getRightEncoderRotations() {
        return m_rightEncoder.getPosition().getValueAsDouble() - m_rightEncoderOffsetRotations;
    }

    /**
     * Returns the distance traveled by the right side in meters.
     */
    @Logged
    public double getRightDistanceMeters() {
        return getRightEncoderRotations() * kDistancePerRotationMeters;
    }

    /**
     * Returns the distance traveled by the left side in meters. There is no left encoder, so
     * this is a "virtual" reading reconstructed from the right encoder and the gyro.
     *
     * <p>For a differential drive, heading change = (rightDistance - leftDistance) / trackWidth
     * (radians, CCW-positive). Solving for the left side:
     * leftDistance = rightDistance - trackWidth * headingChange.
     */
    @Logged
    public double getLeftDistanceMeters() {
        double headingChangeRadians = getHeadingRotation2d().getRadians() - m_headingAtResetRadians;
        return getRightDistanceMeters() - kDriveTrackWidthMeters * headingChangeRadians;
    }

    /**
     * Returns the average distance traveled by both sides in meters.
     */
    @Logged
    public double getAverageDistanceMeters() {
        return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
    }

    /** Returns the right side velocity in meters per second. */
    @Logged
    public double getRightVelocityMetersPerSecond() {
        return m_rightEncoder.getVelocity().getValueAsDouble() * kDistancePerRotationMeters;
    }

    /** Returns the left velocity estimate. Note: this is an estimated
     * reading based on the gyro and right side, rather than from an encoder.
     * Same identity as {@link #getLeftDistanceMeters()}, differentiated in time:
     * leftVelocity = rightVelocity - trackWidth * yawRate.
     */
    @Logged
    public double getLeftVelocityMetersPerSecond() {
        double yawRateRadiansPerSecond =
            Math.toRadians(m_pigeon2.getAngularVelocityZWorld().getValueAsDouble());
        return getRightVelocityMetersPerSecond() - kDriveTrackWidthMeters * yawRateRadiansPerSecond;
    }

    /** Returns the current wheel speeds (left is estimated, see {@link #getLeftVelocityMetersPerSecond()}). */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }



    /**
     * Returns the robot heading as a Rotation2d
     * @return
     */
    public Rotation2d getHeadingRotation2d() {
        return m_pigeon2.getRotation2d();
    }

    /**
     * Returns the robot heading as a Rotation2d
     * @return
     */
    @Logged
    public double getHeadingDegrees() {
        return m_pigeon2.getRotation2d().getDegrees();
    }


    /**
     * Resets both drive encoders to zero: the real right encoder, and the virtual left one
     * (by re-baselining the heading it is measured from). Odometry is re-baselined as well
     * so the robot's pose doesn't jump.
     */
    public void resetEncoders() {
        // Software offsets, so both readings are exactly zero immediately after this runs and the
        // baseline handed to odometry below can't disagree with what the next loop reads.
        m_rightEncoderOffsetRotations = m_rightEncoder.getPosition().getValueAsDouble();
        m_headingAtResetRadians = getHeadingRotation2d().getRadians();
        m_odometry.resetPosition(getHeadingRotation2d(), 0.0, 0.0, getPose());
        markMeasurement(); // Back button = "start measuring from here"
    }


    // ------------ Feedforward Control and Testing ----------------- 
     /* Helper methods for setting drive commands in meters per second */
    private final SimpleMotorFeedforward m_feedforward = 
        new SimpleMotorFeedforward(kDriveBase_kS, kDriveBase_kV, kDriveBase_kA); // Replace with your actual constants

    /**
     * Commands the robot to drive at a specific target velocity using only feedforward.
     * 
     * @param targetVelocityMetersPerSecond The desired speed in m/s
     */
    public Command testFeedforwardCommand(double targetVelocityMetersPerSecond) {
        return run(() -> {
            // Calculate the required voltage to achieve the target velocity
            double appliedVoltage = m_feedforward.calculate(targetVelocityMetersPerSecond);

            // Bypass DifferentialDrive and apply the voltage directly to the motor controllers
            setMotorVoltages(appliedVoltage, appliedVoltage);
        })
        .finallyDo(() -> this.stopMotors())
        .withName("testFeedforward");
    }


    
    // ------- Additional Helper Functions
    // helper function: convert speed in meters per second to RPM
    private static double mpsToMotorRpm(double mps) {
        return mps / kDistancePerRotationMeters * kDriveGearRatio * 60.0;
    }

    private void setSpeeds(double leftVelocityMetersPerSecond, double rightVelocityMetersPerSecond) {
        double leftVoltage = m_feedforward.calculate(leftVelocityMetersPerSecond);
        double rightVoltage = m_feedforward.calculate(rightVelocityMetersPerSecond);

        setMotorVoltages(leftVoltage, rightVoltage);
    }


    // velocity feedback on top of the feedforward, to correct for what the feedforward misses
    private final PIDController m_leftVelocityPid =
        new PIDController(kDriveBase_kP, kDriveBase_kI, kDriveBase_kD);
    private final PIDController m_rightVelocityPid =
        new PIDController(kDriveBase_kP, kDriveBase_kI, kDriveBase_kD);

    /** Drives each side at the requested speed using feedforward plus velocity PID. */
    private void setSpeeds(DifferentialDriveWheelSpeeds differentialDriveWheelSpeeds) {

        double leftVelocityMetersPerSecond = differentialDriveWheelSpeeds.leftMetersPerSecond;
        double rightVelocityMetersPerSecond = differentialDriveWheelSpeeds.rightMetersPerSecond;

        double leftVoltage = m_feedforward.calculate(leftVelocityMetersPerSecond)
            + m_leftVelocityPid.calculate(getLeftVelocityMetersPerSecond(), leftVelocityMetersPerSecond);
        double rightVoltage = m_feedforward.calculate(rightVelocityMetersPerSecond)
            + m_rightVelocityPid.calculate(getRightVelocityMetersPerSecond(), rightVelocityMetersPerSecond);

        setMotorVoltages(leftVoltage, rightVoltage);
    }

    /** Returns robot-relative speeds: forward from the wheels, turn rate from the wheel-speed difference. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    /** Drives at the requested robot-relative speeds (what PathPlanner outputs). */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        setSpeeds(m_kinematics.toWheelSpeeds(speeds));
    }

   

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        // If nothing drove the motors last loop (e.g. an auto is shooting between paths, so it
        // owns the drivetrain but isn't driving it), hold still. Otherwise Motor Safety times
        // out and floods the log with "Output not updated often enough".
        if (!m_drivenThisLoop) {
            stopMotors();
        }
        m_drivenThisLoop = false;

        // update odometry
        m_virtualLeftEncoderReading = getLeftDistanceMeters();
        Pose2d pose = m_odometry.update(
            getHeadingRotation2d(), m_virtualLeftEncoderReading, getRightDistanceMeters());
        m_field.setRobotPose(pose);

        // dashboard values for checking odometry against tape-measure ground truth
        SmartDashboard.putNumber("Drive/Pose X (m)", pose.getX());
        SmartDashboard.putNumber("Drive/Pose Y (m)", pose.getY());
        SmartDashboard.putNumber("Drive/Pose Heading (deg)", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Drive/Right Distance (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Drive/Right Encoder Rotations", getRightEncoderRotations());
        SmartDashboard.putNumber("Drive/Virtual Left Distance (m)", m_virtualLeftEncoderReading);
        SmartDashboard.putNumber("Drive/Gyro Heading (deg)", getHeadingDegrees());

        // Measurement since the last mark (Back button, Start button, or Measure/Mark Here).
        // "Forward" and "Sideways" are relative to the way the robot faced at the mark, so after
        // pushing it 3 m straight ahead you'd expect Forward = 3.00 and Sideways = 0.00.
        m_pathLengthSinceMarkMeters += pose.getTranslation().getDistance(m_previousPose.getTranslation());
        m_previousPose = pose;
        Pose2d sinceMark = pose.relativeTo(m_markPose);
        SmartDashboard.putNumber("Measure/Forward (m)", sinceMark.getX());
        SmartDashboard.putNumber("Measure/Sideways (m)", sinceMark.getY());
        SmartDashboard.putNumber("Measure/Straight-line Distance (m)", sinceMark.getTranslation().getNorm());
        SmartDashboard.putNumber("Measure/Path Length (m)", m_pathLengthSinceMarkMeters);
        SmartDashboard.putNumber("Measure/Turned (deg)", getHeadingDegrees() - m_markGyroDegrees);

        // If either of these is false the device isn't answering on CAN, and its readings are
        // meaningless: a dead gyro reports a heading that never changes.
        SmartDashboard.putBoolean("Drive/Gyro Connected", m_pigeon2.isConnected());
        SmartDashboard.putBoolean("Drive/Encoder Connected", m_rightEncoder.isConnected());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
        // Push the commanded voltages through the drivetrain physics model...
        double batteryVolts = RobotController.getBatteryVoltage();
        m_driveSim.setInputs(
            MathUtil.clamp(m_leftCommandedVolts, -batteryVolts, batteryVolts),
            MathUtil.clamp(m_rightCommandedVolts, -batteryVolts, batteryVolts));
        m_driveSim.update(kRobotLoopPeriod);

        // ...then write the results into the simulated sensors, so odometry reads them exactly
        // like it reads the real CANcoder and Pigeon2 (there is no left encoder here either).
        CANcoderSimState encoderSim = m_rightEncoder.getSimState();
        encoderSim.setSupplyVoltage(batteryVolts);
        encoderSim.setRawPosition(m_driveSim.getRightPositionMeters() / kDistancePerRotationMeters);
        encoderSim.setVelocity(m_driveSim.getRightVelocityMetersPerSecond() / kDistancePerRotationMeters);

        Pigeon2SimState gyroSim = m_pigeon2.getSimState();
        gyroSim.setSupplyVoltage(batteryVolts);
        gyroSim.setRawYaw(m_driveSim.getHeading().getDegrees());
        gyroSim.setAngularVelocityZ(Math.toDegrees(
            (m_driveSim.getRightVelocityMetersPerSecond() - m_driveSim.getLeftVelocityMetersPerSecond())
                / kDriveTrackWidthMeters));
    }


   
}
