package frc.robot.commands;

import static frc.robot.Constants.OdometryConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.Optional;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.HopperStartView;
import frc.robot.vision.Limelight.TargetMeasurement;

class HopperAutoStartTest {
    private static final AtomicBoolean RED = new AtomicBoolean();
    private static EstimatorDrive drive;
    private static boolean shooting;
    private static int visionAttempts;
    private static int pathOutputs;
    private static final HopperStartView START_VIEW = new HopperStartView(true);

    @BeforeAll
    static void configureRealPathPlannerWithStationaryDrive() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        drive = new EstimatorDrive();
        AutoBuilder.configure(drive::getPose, drive::resetOdometry, drive::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> pathOutputs++, new PPLTVController(0.02),
            RobotConfig.fromGUISettings(), RED::get, drive);
        NamedCommands.registerCommand("hopperStart", Autos.hopperStart(
            drive, drive, DriveSubsystem::getHopperCenterPose).andThen(
                new CaptureHopperStartViewCommand(drive, drive, START_VIEW, DriveSubsystem::getHopperCenterPose,
                    () -> RED.get() ? 10 : 26, () -> drive.now, () -> true)));
        // Pause at the first shot so no later path or return command can hide a startup error.
        NamedCommands.registerCommand("shoot", Commands.run(() -> shooting = true));
        NamedCommands.registerCommand("visionAlign", Commands.runOnce(() -> visionAttempts++));
        for (String name : new String[] {"intake", "postTrenchRelocalize", "resetBack", "resetBack2x", "resetBack3x"}) {
            NamedCommands.registerCommand(name, Commands.none());
        }
    }

    @AfterAll
    static void restoreBlueAlliance() {
        RED.set(false);
    }

    @Test
    void everyCenterAutoSeedsAnExactSquarePoseBeforeShootingOnBothAlliances() {
        for (boolean red : new boolean[] {false, true}) {
            RED.set(red);
            for (String name : new String[] {"Center Left", "Center Left 2x", "Center Left 3x",
                    "Center Right", "Center Right 2x", "Center Right 3x"}) {
                assertTrue(Autos.startsAtHopperCenter(name));
                drive.estimator.resetPosition(drive.rawGyro, 1.4, 2.5,
                    new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(37)));
                drive.resets = 0;
                drive.commanded = new ChassisSpeeds(0.2, 0, 0.3);
                shooting = false;
                visionAttempts = 0;
                pathOutputs = 0;

                var auto = new PathPlannerAuto(name);
                auto.initialize();
                for (int loop = 0; loop < 60 && !shooting; loop++) {
                    auto.execute();
                    drive.estimator.update(drive.rawGyro, 1.4, 2.5);
                    drive.now += 0.02;
                }

                Pose2d blue = new Pose2d(kHopperCenterXMeters, kHopperTagCenterYMeters,
                    Rotation2d.fromDegrees(180));
                Pose2d expected = red ? FlippingUtil.flipFieldPose(blue) : blue;
                assertEquals(expected.getX(), drive.getPose().getX(), 1e-9, name);
                assertEquals(expected.getY(), drive.getPose().getY(), 1e-9, name);
                assertEquals(0.0, MathUtil.angleModulus(drive.getPose().getRotation().getRadians()
                    - expected.getRotation().getRadians()), 1e-9, name);
                assertEquals(1, drive.resets, "Only the known-pose command may reset startup odometry");
                assertEquals(71.0, drive.rawGyro.getDegrees(), 1e-9, "Keep the raw gyro; reset its field reference");
                assertEquals(0.0, drive.commanded.vxMetersPerSecond);
                assertEquals(0.0, drive.commanded.omegaRadiansPerSecond);
                assertTrue(shooting, "Known placement must release the auto directly to shooting");
                assertEquals(0, visionAttempts, "Initial vision must not replace the known starting angle");
                assertEquals(0, pathOutputs, "No path movement before the initial shot");
                var saved = START_VIEW.getReference().orElseThrow();
                assertEquals(red ? 10 : 26, saved.tagId());
                assertEquals(frc.robot.vision.HopperViewAlignment.photo().mainX(), saved.imageHorizontalDegrees(), 1e-9);
                assertEquals(expected, saved.startingPose());
                assertFalse(drive.active, "Startup capture must release vision before shooting");
                auto.end(true);
            }
        }
        assertFalse(Autos.startsAtHopperCenter("drive fwd"));
        assertFalse(Autos.startsAtHopperCenter("right side"));
    }

    private static class EstimatorDrive extends HopperTagAlignmentCommandTest.FakeDrive {
        final Rotation2d rawGyro = Rotation2d.fromDegrees(71);
        final DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(
            new DifferentialDriveKinematics(0.53), rawGyro, 1.4, 2.5, Pose2d.kZero);
        int resets;

        EstimatorDrive() {
            super(Pose2d.kZero);
        }

        @Override
        public Pose2d getPose() {
            return estimator.getEstimatedPosition();
        }

        @Override
        public void resetOdometry(Pose2d pose) {
            estimator.resetPosition(rawGyro, 1.4, 2.5, pose);
            resets++;
        }

        @Override
        public Optional<TargetMeasurement> getNewHopperTagPose(int tagId) {
            if (now - lastFrame < 0.039) {
                return Optional.empty();
            }
            lastFrame = now;
            // An independent PnP yaw error must never tilt the trusted startup heading.
            var photo = frc.robot.vision.HopperViewAlignment.photo();
            return Optional.of(new TargetMeasurement(tagId, 0, kHopperTagStandOffMeters,
                now - 0.02, RED.get() ? -5 : 175, -0.04, 0.4, photo.mainX(), photo.companionX(), true,
                photo.mainY(), photo.companionY()));
        }
    }
}
