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
        public static final double kDriveGearRatio = 8.45; //10.71 was old value
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
         *
         * <p>Re-measured 2026-10-19 at 0.9331: moving the robot exactly 2.00 m reported 1.93 m,
         * i.e. 3.5% short, so 0.9331 * (2.00 / 1.93) = 0.9669. That is 0.4721 m per encoder
         * rotation, an effective wheel diameter of 0.1503 m (5.92 in).
         */
        public static final double kEncoderDistanceCalibration = 0.9669;
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

        // Shooting agitation: keep every target ahead of where shooting started so the robot
        // moves away from the hopper first, and never targets a position behind that start.
        public static final double kShootWiggleForwardMeters = 0.02;
        public static final double kShootWiggleReturnMeters = 0.008;
        public static final double kShootWiggleMaxSpeedMetersPerSecond = 0.12;
        /** Each direction gets 0.22 s of motion followed by 0.13 s of rest. */
        public static final double kShootWiggleIntervalSeconds = 0.35;
        public static final double kShootWiggleMoveSeconds = 0.22;
        public static final double kShootWigglePositionP = 8.0;
        public static final double kShootWiggleHeadingP = 2.0;
        public static final double kShootWiggleMaxTurnRateRadPerSec = 0.20;

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

        /** Bumper-to-bumper robot length, front to back (meters), measured on the real robot. */
        public static final double kRobotLengthMeters = 0.83;
        /**
         * X of the hub's face toward the blue alliance wall (meters), from WPILib's 2026 AprilTag
         * layout (tags 25/26). The hub is centered at Y = 4.035.
         */
        public static final double kHubNearFaceXMeters = 4.022;
        /** Hub center Y (meters), halfway between its side tags. */
        public static final double kHubCenterYMeters = 4.035;
        /** Gap from the robot's front bumper to the hub face at the starting spot (meters). */
        public static final double kStartingGapToHubMeters = 2.0;

        /**
         * Starting X position of the robot's CENTER on the field (meters): front bumper
         * kStartingGapToHubMeters short of the hub face, so 4.022 - 2.0 - 0.415 = 1.607.
         */
        public static final double kStartingXMeters =
            kHubNearFaceXMeters - kStartingGapToHubMeters - kRobotLengthMeters / 2.0;
        /** Starting Y position of the robot's center (meters): lined up with the hub's center. */
        public static final double kStartingYMeters = kHubCenterYMeters;
        /** Starting heading of the robot on the field (degrees, CCW-positive). */
        public static final double kStartingHeadingDegrees = 0.0;

        // Pose the Back button puts the robot at: PathPlanner's linked waypoint "Hopper Center"
        // (start of the "hopper front test" paths, end of "reset back"). If that waypoint is
        // moved in the PathPlanner GUI, copy its new X/Y here.
        /** X position of the "Hopper Center" waypoint (meters, blue side). */
        public static final double kHopperCenterXMeters = 3.42587253414264;
        /** Center of AprilTag 26 in WPILib's installed 2026 welded field layout. */
        public static final double kHopperTagFaceXMeters = 4.0218614;
        public static final double kHopperTagCenterYMeters = 4.0346376;
        /** Known placement is centered on the hopper, square with its back against the face. */
        public static final double kHopperDriverStationLeftOffsetMeters = 0.0;
        public static final double kHopperCenterYMeters =
            kHopperTagCenterYMeters + kHopperDriverStationLeftOffsetMeters;
        /** Tag coordinates of the nominal path waypoint; independent of the camera aiming target. */
        public static final double kHopperPathRightReferenceMeters = -kHopperDriverStationLeftOffsetMeters;
        /** Robot-center distance out from the tag face; retains the existing bumper clearance. */
        public static final double kHopperTagStandOffMeters = kHopperTagFaceXMeters - kHopperCenterXMeters;
        /** Heading at "Hopper Center": front facing directly away from the hopper, toward the alliance wall. */
        public static final double kHopperCenterHeadingDegrees = 180.0;
    }

    public static class VisionConstants {
        /** Name of the Limelight, as set in its web interface (default "limelight"). */
        public static final String kLimelightName = "limelight";
        public static final int kBlueHopperTagId = 26;
        public static final int kBlueHopperCompanionTagId = 25;
        /** Rotationally corresponding tag on the red hub face. */
        public static final int kRedHopperTagId = 10;
        public static final int kRedHopperCompanionTagId = 9;
        public static final double kHopperPairMinImageSeparationDegrees = 2.0;
        public static final int kHopperTagAcquireFrames = 3;
        public static final int kHopperTagConfirmFrames = 3;
        public static final double kHopperStartViewCaptureTimeoutSeconds = 0.8;
        public static final double kHopperStartViewMaxRangeSpreadMeters = 0.03;
        public static final double kHopperStartViewMaxAngleSpreadDegrees = 1.0;
        public static final double kHopperImageCenterToleranceDegrees = 1.5;
        /** Factory LL4 lens; the uploaded reference image uses the full 1280x800 view. */
        public static final double kHubPhotoHorizontalFovDegrees = 82.0;
        public static final double kHubPhotoVerticalFovDegrees = 56.2;
        public static final double kHubPhotoCaptureToleranceDegrees = 5.0;
        public static final double kHopperViewFinePositionToleranceMeters = 0.01;
        public static final double kHopperViewFineStopMarginMeters = 0.005;
        public static final double kHopperTagAcquireTimeoutSeconds = 1.2;
        public static final double kHopperTagMaxSpreadMeters = 0.08;
        public static final double kHopperTagMaxLateralMeters = 1.0;
        public static final double kHopperTagMaxRangeMeters = 3.0;
        /**
         * How much to trust a vision position, as a standard deviation (meters): bigger means
         * trust it less and move the pose toward it more gently. 0.7 is Limelight's suggested
         * value for MegaTag2; the wheel odometry's own default is 0.02.
         */
        public static final double kVisionXYStdDevMeters = 0.7;
        /**
         * Ignore vision while the robot turns faster than this (deg/s): MegaTag2 relies on the
         * heading we send it, which lags slightly during fast turns and smears the position.
         */
        public static final double kMaxYawRateDegreesPerSecond = 720.0;
        /** Ignore results whose tags are farther than this on average (meters); far tags are noisy. */
        public static final double kMaxTagDistanceMeters = 4.0;
        /**
         * Ignore vision positions farther than this from where odometry thinks the robot is
         * (meters). Real drift over an auto is well under this, so anything bigger means the
         * vision is wrong (wrong tag IDs or field map, bad camera pose), not the odometry.
         */
        public static final double kMaxVisionJumpMeters = 1.0;
        /**
         * The general-purpose vision pose reset averages this many accepted frames before snapping
         * the pose to them (about 0.17 s at the Limelight's 30 fps)...
         */
        public static final int kVisionResetFrames = 5;
        /** ...and still snaps with at least this many if it times out... */
        public static final int kVisionResetMinFrames = 3;
        /** ...after this long (seconds). With fewer frames (tags not in view) it leaves the pose alone. */
        public static final double kVisionResetTimeoutSeconds = 0.5;
        /** Reject old camera frames, including the last frame left behind after a disconnect. */
        public static final double kMaxVisionAgeSeconds = 0.3;
        /** Allow a small timestamp skew between NetworkTables and the FPGA clock. */
        public static final double kVisionFutureToleranceSeconds = 0.05;
        /** Trench markers on both sides of the blue and red trenches in the 2026 field map. */
        public static final java.util.Set<Integer> kTrenchAprilTagIds = java.util.Set.of(1, 6, 7, 12, 17, 22, 23, 28);
        public static final double kTrenchVisionMaxDistanceMeters = 3.0;
        public static final double kTrenchVisionMaxAmbiguity = 0.2;
        public static final double kTrenchVisionMaxYawRateDegreesPerSecond = 120.0;
        /** Apply a confirmed trench position promptly so PathPlanner can correct its track. */
        public static final double kTrenchVisionXYStdDevMeters = 0.01;
        /** Deliberate localization stop after the trench route, before the reset-back path. */
        public static final double kPostTrenchVisionTimeoutSeconds = 1.0;
        public static final int kPostTrenchVisionFrames = 5;
        /** Every pair of vision-minus-odometry offsets must agree within this distance. */
        public static final double kPostTrenchVisionMaxSpreadMeters = 0.12;
        /** Validated trench-tag consensus can recover this much drift, including from one tag. */
        public static final double kPostTrenchVisionMaxCorrectionMeters = 2.0;
        /** Field size (meters), from WPILib's 2026 AprilTag layout; vision positions outside it are rejected. */
        public static final double kFieldLengthMeters = 16.541;
        public static final double kFieldWidthMeters = 8.069;
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

        // "Drive 3 m" auto (DriveSubsystem.driveDistanceCommand): speeds up, cruises, and slows to
        // a stop on the target instead of cutting the motors there and coasting past it.
        /** How far the "Drive 3 m" auto drives straight forward (meters). */
        public static final double kDriveStraightDistanceMeters = 3.0;
        /**
         * Top speed for the drive-distance auto (m/s). Kept low so the wheels don't spin or skid,
         * which would make the encoder count distance the robot didn't actually travel.
         */
        public static final double kDriveStraightMaxSpeedMetersPerSecond = 1.0;
        /** How quickly the drive-distance auto speeds up and slows down (m/s^2); gentle for the same reason. */
        public static final double kDriveStraightMaxAccelMetersPerSecondSq = 1.0;
        /** Extra speed (m/s) added per meter the robot is behind where it should be by now. */
        public static final double kDriveStraightDistanceP = 2.0;
        /** Turn rate (rad/s) per radian the robot has turned away from its starting heading. */
        public static final double kDriveStraightHeadingP = 2.0;
        /** The drive-distance auto finishes once the robot is within this distance of the target (meters)... */
        public static final double kDriveStraightToleranceMeters = 0.02;
        /** ...and moving slower than this (m/s), so it doesn't end while still rolling. */
        public static final double kDriveStraightSpeedToleranceMetersPerSecond = 0.05;

        /** Maximum X/Y distance from Hopper Center when alignment finishes (meters). */
        public static final double kHopperAlignToleranceMeters = 0.03;
        /** Finish the sideways correction before accepting the final stand-off position. */
        public static final double kHopperAlignLateralToleranceMeters = 0.012;
        /** Small autonomous-only trim toward Driver Station left to counter the observed right bias. */
        public static final double kHopperReturnLeftTrimMeters = 0.0;
        /** Final facing direction and initial turn must be within this angle (radians). */
        public static final double kHopperAlignHeadingToleranceRadians = Math.toRadians(0.5);
        /** Slow corrections near the hopper, using forward or reverse as needed. */
        public static final double kHopperAlignMaxSpeedMetersPerSecond = 0.4;
        public static final double kHopperAlignMaxTurnRateRadPerSec = 1.0;
        public static final double kHopperAlignPositionP = 1.5;
        public static final double kHopperAlignHeadingP = 3.0;
        /** Stop translating and turn again if a new vision correction changes the bearing this far. */
        public static final double kHopperAlignReorientRadians = Math.toRadians(45.0);
        public static final double kHopperAlignTurnSpeedToleranceRadPerSec = 0.1;
        /** Bound the whole correction, including turns and settling, if the robot cannot reach the spot. */
        public static final double kHopperAlignTimeoutSeconds = 10.0;
        /** Restoring the startup view may require extra forward clearance at the same slow speed. */
        public static final double kHopperReturnAlignTimeoutSeconds = 18.0;
        /** Make room to steer a reverse arc if lateral correction is needed close to the hopper. */
        public static final double kHopperAlignClearanceMeters = 0.65;
        public static final double kHopperAlignClearanceSpeedMetersPerSecond = 0.35;
        public static final double kHopperAlignBearingP = 4.0;
        public static final double kHopperAlignFinalBearingP = 4.0;
        /** Keep the rear-facing camera aimed at the hub throughout the correction arc. */
        public static final double kHopperApproachMaxHeadingRadians = Math.toRadians(30.0);
        public static final double kHopperApproachMaxSpeedMetersPerSecond = 0.25;
        public static final double kHopperApproachBrakingAccelMetersPerSecondSq = 0.6;
        public static final double kHopperApproachStopMarginMeters = 0.02;

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
