// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        /** Stick values smaller than this are treated as zero, to stop drift from off-center sticks. */
        public static final double kDriveDeadband = 0.08;
        /** How far a trigger must be pulled to count as pressed. */
        public static final double kTriggerThreshold = 0.5;
    }

    public static class RobotConstants {
        public static final double kRobotLoopPeriod = 0.02; // 0.02 s -> 50 Hz
    }


    public static class DriveConstants {
        public static final int kLeftLeaderId = 11;
        public static final int kLeftFollowerId = 8;
        public static final int kRightLeaderId = 10;
        public static final int kRightFollowerId = 7;

        public static final int kRightEncoderID = 4;
        public static final int kPigeon2ID = 5;

        public static final boolean kLeftLeaderReversed = true;
        public static final boolean kRightLeaderReversed = false;

        public static final int kDriveMotorCurrentLimit = 60;

        /** Wheel diameter in meters (e.g. 6 in ≈ 0.1524 m). */
        public static final double kWheelDiameterMeters = 0.1554;
        /**
         * Gear ratio motor-to-wheel (e.g. 8.45 for KitBot). NOTE: this does NOT affect odometry —
         * the CANcoder is on the wheel shaft, so distance depends only on wheel size and
         * kEncoderDistanceCalibration below. Changing this alone will not change measured distance.
         */
        public static final double kDriveGearRatio = 10.71; //10.71;
        /**
         * Correction applied to the distance the encoder reports, for whatever the wheel-size
         * number alone doesn't capture (tread wear, an encoder not exactly 1:1 with the wheel).
         * 1.0 means one encoder rotation is exactly one wheel circumference.
         *
         * <p>To calibrate in one measurement:
         * <ol>
         *   <li>Press Start (resets the encoder), then push the robot in a straight line a
         *       distance you measure with a tape — 3 m or more is best.
         *   <li>Read "Drive/Right Encoder Rotations" on the dashboard — call it R.
         *   <li>True meters per rotation = (measured distance) / R. Set this constant to
         *       that value divided by (pi * kWheelDiameterMeters).
         * </ol>
         * <p>Quicker version once a value is already close: new = old * (real / reported).
         *
         * Measured 2026-09-23 with this factor at 1.15: pushing the robot 2.84 m reported 3.50 m,
         * i.e. 23% too far, so 1.15 * (2.84 / 3.50) = 0.9331. That works out to 0.4556 m per
         * encoder rotation, an effective wheel diameter of 0.145 m (5.71 in) rather than the
         * nominal 6 in — about what worn tread on a 6 in wheel measures.
         */
        public static final double kEncoderDistanceCalibration = 0.9331;
        public static final double kDistancePerRotationMeters =
            Math.PI * kWheelDiameterMeters * kEncoderDistanceCalibration; // encoder directly on wheel shaft
        
        /**
         * Effective track width (meters): how far apart the wheels behave as if they are, which on
         * a skid-steer is not the same as the measured center-to-center distance, because the
         * wheels scrub sideways when turning. Used both to turn the robot (converting a requested
         * turn rate into a left/right speed difference) and to reconstruct the virtual left
         * encoder from the gyro.
         *
         * <p>This is the measured center-to-center wheel spacing. Tune it by turning the robot and
         * comparing the angle it actually turns against what was asked for: if it over-rotates,
         * lower this; if it under-rotates, raise it.
         */
        public static final double kDriveTrackWidthMeters = 0.55;
        /**
         * Voltage each side is driven at (in opposite directions) while measuring track width
         * with the "Measure/Find Track Width" button: slow enough to limit wheel scrub, well
         * above the ~1 V needed to get moving (kDriveBase_kS).
         */
        public static final double kTrackWidthSpinVolts = 3.0;

        // feedforward gains determined from SysId
        public static final double kDriveBase_kS = 1.008;
        public static final double kDriveBase_kV = 2.575;
        public static final double kDriveBase_kA = 0.675;

        // pid control parameters: 
        public static final double kDriveBase_kP = 3.21; // possibly up to 4.0
        public static final double kDriveBase_kD = 0.0;
        public static final double kDriveBase_kI = 0.0;  

        /**
         * Scalar applied to encoder-only turn distance calculations.
         * If the robot over-rotates, decrease this; if it under-rotates, increase it.
         */
        public static final double kAutoTurnDistanceScalar = 0.125;

        /**
         * Set true if the right CANcoder counts negative when the robot drives forward.
         * Check this first: push the robot forward by hand and watch "Drive/Right Distance (m)"
         * on the dashboard — it must increase.
         */
        public static final boolean kRightEncoderReversed = false;
    }

    public static class OdometryConstants {
        // Field coordinates: (0, 0) is the bottom-left corner of the field, +X points away from
        // the blue alliance wall, +Y points left, and heading is CCW-positive (0 deg = facing +X).

        /** Starting X position of the robot on the field (meters). */
        public static final double kStartingXMeters = 2.0;
        /** Starting Y position of the robot on the field (meters). */
        public static final double kStartingYMeters = 4.0;
        /** Starting heading of the robot on the field (degrees, CCW-positive). */
        public static final double kStartingHeadingDegrees = 0.0;
    }

    public static class IOConstants {
        public static final int kFlywheelMotorID = 9;
        public static final int kIntakeMotorID = 12;
        public static final int kLoaderMotorID = 19;

        // ======= flywheel constants ==========
        public static final int kFlywheelDefaultTargetRPM = 3000;
        public static final int kFlywheelMaxRPM = 7000;
        /*
         * Shot speeds. The button layout comes from Vancouver2526, but its RPMs were measured on a
         * different flywheel (brushless SparkMax, ~5676 RPM free) than this one (Kraken X60 /
         * TalonFX, ~6000 RPM free); neither has flywheel gearing, so both numbers are motor speed.
         * Its main shot was 1300 of its own 5676, so its speeds are scaled here by this robot's
         * own main shot (kFlywheelDefaultTargetRPM, 3000) over 1300, i.e. x2.31.
         */
        /** Flywheel speed for the left-trigger (main) shot — unchanged from this robot's original default. */
        public static final int kFlywheelLaunchRPM = kFlywheelDefaultTargetRPM;
        /** Flywheel speed for the A-button (high) shot (Vancouver's 1900, scaled). */
        public static final int kFlywheelHighShotRPM = 4400;
        /**
         * Flywheel speed for the B / right-bumper (long-range) shot. Vancouver asked for 6000 on a
         * motor whose free speed is 5676, i.e. "flat out"; the same intent here is this flywheel's
         * own free speed.
         */
        public static final int kFlywheelUltraShotRPM = 6000;
        /** Flywheel speed for the X-button pre-spin toggle (Vancouver's 500, scaled). */
        public static final int kFlywheelSpinUpRPM = 1150;
        /**
         * The flywheel counts as "up to speed" once it reaches this fraction of its target, and the
         * loader only feeds after that (0.8 = 80%, same as Vancouver2526).
         */
        public static final double kFlywheelAtSpeedFraction = 0.8;
        public static final int kFlywheelMotorCurrentLimit = 60;
        // feedforward gains
        public static final double kFlywheel_kS = 0.06;
        public static final double kFlywheel_kV = 0.10;
        public static final double kFlywheel_kA = 0.025;

        // feedback PID controller parameters
        public static final double kFlywheel_kP = 0.13;
        public static final double kFlywheel_kI = 0.0;
        public static final double kFlywheel_kD = 0.0;

        // ======= intake constants ==========
        public static final int kIntakeMotorCurrentLimit = 60;
        /** Open-loop duty cycle ([-1, 1]) used for both runIntakeCommand() and reverseIntakeCommand(). */
        public static final double kIntakeDefaultOutput = 0.3;
        /**
         * Reserved for a future closed-loop conversion of the intake (see IntakeClass javadoc) —
         * not used yet.
         */
        public static final int kIntakeDefaultTargetRPM = 1600;

        // =========== loader constants =========
        public static final int kLoaderMotorCurrentLimit = 60;

        /**
         * Within the loader's own three speeds, faster stages come later in the direction of
         * travel — each stage pulls a game piece away from the one before it rather than letting
         * it back up (a common source of jams): kLoaderToIntakeOutput (0.3) < kLoaderFromIntakeOutput
         * (0.5) < kLoaderToFlywheelOutput (0.6). (Duty cycle isn't directly comparable to the
         * intake's own kIntakeDefaultOutput — different motor, gearing, and roller friction — so
         * this ordering is about the loader's three speeds relative to each other, not to intake.)
         * Values below are from bench testing.
         */
        /** Open-loop duty cycle ([-1, 1]) while receiving a game piece from the intake. */
        public static final double kLoaderFromIntakeOutput = 0.7;
        /** Open-loop duty cycle ([-1, 1]) while sending a game piece back out through the intake. */
        public static final double kLoaderToIntakeOutput = 0.35;
        /** Open-loop duty cycle ([-1, 1]) while feeding a game piece into the flywheel. */
        public static final double kLoaderToFlywheelOutput = 0.4;
        /**
         * Reserved for a future closed-loop conversion of the loader (same pattern as the intake) —
         * not used yet.
         */
        public static final int kLoaderIntakeTargetRPM = 800;
        public static final int kLoaderToFlywheelTargetRPM = 800;
    }

    public static class AutoConstants {
        public static final double kDistanceTargetMeters = 2.0;

        /** Loop period (seconds) used by PathPlanner's LTV differential drive controller. */
        public static final double kLTVDtSeconds = RobotConstants.kRobotLoopPeriod;

        // Named-command timings. Auto commands must end on their own, and the mechanism
        // commands run until interrupted, so each one gets a timeout here.
        /**
         * Longest the auto "shoot" waits for the flywheel to get up to speed before feeding anyway,
         * so a slow flywheel can't stall the whole auto (seconds).
         */
        public static final double kShooterMaxSpinUpSeconds = 2.0;
        /** Time to run the loader into the flywheel once it's up to speed (seconds). */
        public static final double kShooterFeedSeconds = 1.5;
        /** Maximum time the "intake" named command runs if nothing interrupts it (seconds). */
        public static final double kIntakeTimeoutSeconds = 3.0;
    }
}
