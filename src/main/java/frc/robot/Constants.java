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
        public static final double kWheelDiameterMeters = 0.1524;
        /** Gear ratio motor-to-wheel (e.g. 8.45 for KitBot). */
        public static final double kDriveGearRatio = 8.45; //10.71;
        public static final double kDistancePerRotationMeters = 
            Math.PI * kWheelDiameterMeters; // encoder directly on wheel shaft
        
        /**
         * Approximate track width (meters) used for encoder-only in-place turns.
         * Measure center-to-center distance between left and right wheels for best
         * results.
         */    
        public static final double kDriveTrackWidthMeters = 0.55;

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
    }

    public static class IOConstants {
        public static final int kFlywheelMotorID = 9;
        public static final int kIntakeMotorID = 12;
        public static final int kLoaderMotorID = 19;

        // ======= flywheel constants ==========
        public static final int kFlywheelDefaultTargetRPM = 3000;
        public static final int kFlywheelMaxRPM = 7000;
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
        public static final int kIntakeDefaultTargetRPM = 600;

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
    }
}
