package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.OdometryConstants.*;
import static frc.robot.Constants.VisionConstants.kHopperTagAcquireTimeoutSeconds;
import static org.junit.jupiter.api.Assertions.*;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.vision.Limelight.TargetMeasurement;
import frc.robot.vision.HopperViewAlignment;

class HopperTagAlignmentCommandTest {
    private static final Pose2d GOAL = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));
    private static final Pose2d CAMERA_GOAL = new Pose2d(kHopperCenterXMeters, kHopperTagCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));
    private static final Pose2d TAG_REFERENCE = new Pose2d(kHopperCenterXMeters, kHopperTagCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));

    @Test
    void restoresBothTagsToThePhotoWhileTheGyroHoldsHeadingDespiteCameraYawAndPositionBias() {
        // Exercise both the old long approach and the shortened path handoff, with
        // drift to either side, wheel lag and a biased camera position/yaw estimate.
        for (double[] start : new double[][] {{1.95, -0.25}, {1.95, 0.25}, {2.45, -0.25}, {2.45, 0.25}}) {
            FakeDrive io = new FakeDrive(new Pose2d(start[0], GOAL.getY() + start[1],
                GOAL.getRotation().plus(Rotation2d.fromDegrees(10))));
            configurePhotoCamera(io);
            io.visionLateralBiasMeters = 0.18;
            io.independentHeadingBiasDegrees = 30;
            io.rightwardDriftPerMeter = 0.10;
            HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                () -> io.now, () -> io.enabled, kHopperReturnCruiseSpeedMetersPerSecond, null, 0.0, true);
            command.initialize();
            StringBuilder trace = new StringBuilder();
            for (int step = 0; step < 930 && !command.isFinished(); step++) {
                command.execute();
                if (step % 50 == 0) {
                    trace.append("\n").append(io.now).append(" actual=").append(io.actual)
                        .append(" odom=").append(io.odometry).append(" vx=").append(io.commanded.vxMetersPerSecond)
                        .append(" w=").append(io.commanded.omegaRadiansPerSecond).append(" ").append(io.status);
                }
                io.step();
            }
            assertTrue(io.success, io.status + "; actual=" + io.actual + "; tx=" + io.imageAngleDegrees() + trace);
            assertTrue(HopperViewAlignment.photo().matches(io.getNewHopperTagPose(26)
                .orElseGet(() -> { io.now += 0.04; return io.getNewHopperTagPose(26).orElseThrow(); }), 1.5));
            assertEquals(GOAL.getY(), io.actual.getY(), kHopperAlignLateralToleranceMeters);
            assertEquals(0, io.actual.getRotation().minus(GOAL.getRotation()).getRadians(), kHopperAlignHeadingToleranceRadians);
            assertTrue(io.minimumOutward >= kHopperTagStandOffMeters - 0.01, "Do not back through the photographed stand-off");
            command.end(false);
        }
    }

    private static void configurePhotoCamera(FakeDrive io) {
        var photo = HopperViewAlignment.photo();
        double distance = 0.3556 / (Math.tan(Math.toRadians(photo.mainX())) - Math.tan(Math.toRadians(photo.companionX())));
        double right = -distance * Math.tan(Math.toRadians(photo.mainX()));
        io.cameraOffsetRobot = new Translation2d(distance - kHopperTagStandOffMeters, right);
        io.fullView = true;
    }

    @Test
    void bothTagViewCorrectsRightwardMissEvenWhenFieldPositionNeverEntersTheOldFineWindow() {
        for (double offset : new double[] {-0.25, 0.25}) {
            FakeDrive io = new FakeDrive(new Pose2d(2.45, GOAL.getY() + offset, GOAL.getRotation()));
            configurePhotoCamera(io);
            // The image is accurate but the field solve says the robot is farther away
            // and left of its actual location. Following that solve alone misses right.
            io.visionLateralBiasMeters = 0.40;
            io.visionOutwardBiasMeters = 0.35;
            io.rightwardDriftPerMeter = 0.10;
            HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                () -> io.now, () -> io.enabled, kHopperReturnCruiseSpeedMetersPerSecond, null, 0.0, true);
            command.initialize();
            StringBuilder trace = new StringBuilder();
            for (int step = 0; step < 930 && !command.isFinished(); step++) {
                command.execute();
                if (step % 50 == 0) {
                    trace.append("\n").append(io.now).append(" actual=").append(io.actual)
                        .append(" odom=").append(io.odometry).append(" vx=").append(io.commanded.vxMetersPerSecond)
                        .append(" w=").append(io.commanded.omegaRadiansPerSecond).append(" ").append(io.status);
                }
                io.step();
            }
            assertTrue(io.success, io.status + "; actual=" + io.actual + trace);
            assertEquals(GOAL.getY(), io.actual.getY(), kHopperAlignLateralToleranceMeters);
            assertEquals(0, io.actual.getRotation().minus(GOAL.getRotation()).getRadians(),
                kHopperAlignHeadingToleranceRadians);
            assertTrue(io.minimumOutward >= kHopperTagStandOffMeters - 0.01,
                "Biased field range must not send the robot backward through the hopper stand-off");
            command.end(false);
        }
    }

    @Test
    void shiftedLeftHandoffCorrectsDriftFromEitherSide() throws Exception {
        var points = new com.fasterxml.jackson.databind.ObjectMapper().readTree(java.nio.file.Path.of(
            "src/main/deploy/pathplanner/paths/hopper front test L.path").toFile()).get("waypoints");
        var end = points.get(points.size() - 1);
        double x = end.get("anchor").get("x").asDouble();
        double y = end.get("anchor").get("y").asDouble();
        Rotation2d heading = new Rotation2d(x - end.get("prevControl").get("x").asDouble(),
            y - end.get("prevControl").get("y").asDouble());
        for (double drift : new double[] {0.0, -0.35, 0.35}) {
            FakeDrive io = new FakeDrive(new Pose2d(x, y + drift, heading));
            configurePhotoCamera(io);
            io.visionLateralBiasMeters = 0.18;
            io.rightwardDriftPerMeter = 0.10;
            HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                () -> io.now, () -> io.enabled, kHopperReturnCruiseSpeedMetersPerSecond,
                null, kHopperReturnLeftTrimMeters, true);
            command.initialize();
            double minX = io.actual.getX();
            for (int step = 0; step < 930 && !command.isFinished(); step++) {
                command.execute();
                io.step();
                minX = Math.min(minX, io.actual.getX());
            }
            assertTrue(io.success, "drift=" + drift + "; " + io.status + "; actual=" + io.actual);
            assertEquals(GOAL.getY() + kHopperReturnLeftTrimMeters, io.actual.getY(), kHopperAlignLateralToleranceMeters);
            assertEquals(0, io.actual.getRotation().minus(GOAL.getRotation()).getRadians(),
                kHopperAlignHeadingToleranceRadians);
            assertTrue(minX > 1.3, "Keep the clearance maneuver away from the alliance wall");
            assertTrue(io.minimumOutward >= kHopperTagStandOffMeters - 0.01);
            command.end(false);
        }
    }

    @Test
    void unusablePairStopsBetweenFramesWithoutSwitchingToBiasedFieldPoseAndCanRecover() {
        FakeDrive io = new FakeDrive(GOAL);
        var reference = captureStartView(io, GOAL, 26);
        io.actual = new Pose2d(GOAL.getX() - 0.6, GOAL.getY() + 0.15, GOAL.getRotation());
        io.odometry = GOAL;
        io.visionLateralBiasMeters = -0.5;
        var command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperReturnCruiseSpeedMetersPerSecond, reference);
        command.initialize();
        for (int i = 0; i < 50 && !io.acquired; i++) {
            command.execute();
            io.step();
        }
        assertTrue(io.acquired);
        io.invalidPairGeometry = true;
        io.now += 0.04;
        for (int i = 0; i < 12; i++) {
            command.execute();
            assertEquals(0.0, io.commanded.vxMetersPerSecond, 1e-9);
            assertEquals(0.0, io.commanded.omegaRadiansPerSecond, 1e-9);
            assertFalse(command.isFinished(), "A brief bad frame should pause the return, not abandon it");
            assertFalse(io.success);
            io.step();
        }
        io.invalidPairGeometry = false;
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual);
        assertEquals(GOAL.getY(), io.actual.getY(), kHopperAlignLateralToleranceMeters);
        command.end(false);
    }

    @Test
    void unusablePairCannotAcquireByFallingBackToFieldPose() {
        FakeDrive io = new FakeDrive(GOAL);
        var reference = captureStartView(io, GOAL, 26);
        io.invalidPairGeometry = true;
        var command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperReturnCruiseSpeedMetersPerSecond, reference);
        command.initialize();
        double started = io.now;
        run(command, io);
        assertFalse(io.acquired);
        assertFalse(io.success);
        assertFalse(io.moved);
        assertTrue(io.now - started < kHopperTagAcquireTimeoutSeconds + 0.1);
        command.end(false);
    }

    @Test
    void fasterReturnSavesTimeWhileSlowingNearTheHubAndPreservingStandOff() {
        double slowTime = 0.0;
        for (double limit : new double[] {kHopperApproachMaxSpeedMetersPerSecond,
                kHopperReturnCruiseSpeedMetersPerSecond}) {
            FakeDrive io = new FakeDrive(new Pose2d(2.45, GOAL.getY(), GOAL.getRotation()));
            configurePhotoCamera(io);
            HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                () -> io.now, () -> io.enabled, limit, null, 0.0, true);
            double startTime = io.now;
            command.initialize();
            for (int step = 0; step < 930 && !command.isFinished(); step++) {
                command.execute();
                if (GOAL.getX() - io.actual.getX() < kHopperReturnSlowdownDistanceMeters - 0.05) {
                    assertTrue(io.commanded.vxMetersPerSecond >= -kHopperApproachMaxSpeedMetersPerSecond - 1e-9,
                        "Final approach must retain the slow speed even with cruise enabled");
                }
                io.step();
            }
            assertTrue(io.success, io.status);
            assertTrue(io.minimumOutward >= kHopperTagStandOffMeters - 0.01,
                "Faster cruise must still brake before backing into the hopper");
            double elapsed = io.now - startTime;
            if (limit == kHopperApproachMaxSpeedMetersPerSecond) {
                slowTime = elapsed;
            } else {
                assertTrue(io.peakReverseSpeed > kHopperApproachMaxSpeedMetersPerSecond + 0.05);
                assertTrue(io.peakReverseSpeed <= kHopperReturnCruiseSpeedMetersPerSecond + 1e-9);
                assertTrue(elapsed < slowTime - 0.20, "Cruise should shorten the return without speeding up docking");
            }
            command.end(false);
        }
    }

    @Test
    void returnTrimPhysicallyMovesLeftFromEitherSideWithoutAccumulatingAcrossCycles() {
        FakeDrive io = new FakeDrive(GOAL);
        io.cameraOffsetRobot = new Translation2d(-0.20, 0.08);
        io.visionLateralBiasMeters = 0.18;
        var reference = captureStartView(io, GOAL, 26);
        // A changed field-solve bias must not disable image feedback when a trim is set.
        io.visionLateralBiasMeters = 0.0;
        io.visionOutwardBiasMeters = 0.3;
        for (double offset : new double[] {-0.25, 0.25}) {
            io.actual = new Pose2d(1.95, GOAL.getY() + 0.2 + offset, GOAL.getRotation());
            io.odometry = GOAL;
            io.rightwardDriftPerMeter = 0.14;
            HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond,
                reference, kHopperReturnLeftTrimMeters);
            command.initialize();
            run(command, io);
            assertTrue(io.success, io.status + "; actual=" + io.actual);
            assertEquals(GOAL.getY() + 0.20, io.actual.getY(),
                kHopperAlignLateralToleranceMeters);
            assertEquals(GOAL.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
            assertEquals(GOAL, reference.startingPose(), "The known starting placement stays centered");
            command.end(false);
        }
    }

    @Test
    void driverStationLeftTrimMirrorsOnRedWithoutChangingHeadingOrStandOff() {
        Pose2d blue = HopperTagAlignmentCommand.withDriverStationLeftTrim(GOAL, 0.10);
        Pose2d redReference = new Pose2d(13.1, GOAL.getY(), Rotation2d.kZero);
        Pose2d red = HopperTagAlignmentCommand.withDriverStationLeftTrim(redReference, 0.10);
        assertEquals(GOAL.getY() + 0.10, blue.getY(), 1e-9);
        assertEquals(redReference.getY() - 0.10, red.getY(), 1e-9);
        assertEquals(GOAL.getX(), blue.getX(), 1e-9);
        assertEquals(redReference.getX(), red.getX(), 1e-9);
        assertEquals(GOAL.getRotation(), blue.getRotation());
        assertEquals(redReference.getRotation(), red.getRotation());
    }

    @Test
    void tag26AloneCannotStartOrContinueTheReturnCorrection() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX() - 0.8, GOAL.getY() - 0.25, GOAL.getRotation()));
        io.companionAvailable = false;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertFalse(io.acquired);
        assertFalse(io.moved);
        assertTrue(io.status.contains("25 + 26"));
        command.end(false);

        io.companionAvailable = true;
        command.initialize();
        for (int step = 0; step < 30; step++) {
            command.execute();
            io.step();
        }
        assertTrue(io.acquired);
        assertTrue(io.moved);
        io.companionAvailable = false;
        for (int step = 0; step < 20; step++) {
            command.execute();
            io.step();
        }
        assertEquals(0, io.commanded.vxMetersPerSecond);
        assertEquals(0, io.commanded.omegaRadiansPerSecond);
        assertFalse(io.success);
        assertTrue(io.status.contains("waiting for both hopper tags"));
        command.end(true);
    }

    @Test
    void gyroAndPairCorrectActualHeadingAndTranslationEvenWhenTag26ImageSaysCentered() {
        FakeDrive io = new FakeDrive(GOAL);
        var reference = captureStartView(io, GOAL, 26);
        double yaw = Math.atan2(0.20, kHopperTagStandOffMeters);
        io.actual = new Pose2d(GOAL.getX(), GOAL.getY() - 0.20,
            GOAL.getRotation().plus(Rotation2d.fromRadians(yaw)));
        io.odometry = new Pose2d(GOAL.getTranslation(), io.actual.getRotation());
        assertEquals(0, io.imageAngleDegrees(), 1e-9);
        assertNotEquals(reference.companionImageHorizontalDegrees(), io.companionImageAngleDegrees(), 1.5);
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual);
        assertTrue(io.moved);
        assertEquals(GOAL.getY(), io.actual.getY(), kHopperAlignLateralToleranceMeters);
        assertEquals(0, io.actual.getRotation().minus(GOAL.getRotation()).getRadians(), kHopperAlignHeadingToleranceRadians);
        command.end(false);
    }

    @Test
    void physicallyMovesLeftToTag26EvenWhenOdometryClaimsAlreadyCentered() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.25, CAMERA_GOAL.getRotation()));
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual + "; camera=" + io.cameraPosition());
        assertTrue(io.moved, "Must physically move, not only reset odometry");
        assertTrue(io.actual.getY() > CAMERA_GOAL.getY() - 0.04, "Must move left from Driver Station view");
        assertTrue(io.actual.getTranslation().getDistance(CAMERA_GOAL.getTranslation()) <= kHopperAlignToleranceMeters + 0.005);
        command.end(false);
        assertFalse(io.active);
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        assertEquals(0.0, io.commanded.omegaRadiansPerSecond);
    }

    @Test
    void aBadGyroFieldReferenceStopsInsteadOfReseedingHeadingFromTheCamera() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        io.odometry = new Pose2d(CAMERA_GOAL.getTranslation(), Rotation2d.kZero);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertFalse(io.success);
        assertFalse(io.moved);
        assertEquals(0, io.odometry.getRotation().getDegrees(), 1e-9);
        assertTrue(io.status.contains("gyro field heading"));
    }

    @Test
    void cannotDeclareSuccessFromOdometryWithoutFinalTagConfirmation() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        io.loseTagAfterMoving = true;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.moved);
        assertFalse(io.success);
        assertTrue(io.status.contains("timed out"));
    }

    @Test
    void noTagAndDisabledVisionStopWithExplicitFailure() {
        FakeDrive io = new FakeDrive(CAMERA_GOAL);
        io.visible = false;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertFalse(io.success);
        assertFalse(io.moved);
        assertTrue(io.status.contains("no fresh consistent position for hopper tags 25 + 26"));

        io.enabled = false;
        command.initialize();
        command.execute();
        assertTrue(command.isFinished());
        assertTrue(io.status.contains("vision disabled"));
    }

    @Test
    void usesRobotGyroEvenWhenTheCameraHasNoIndependentHeading() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        io.reliableHeading = false;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.moved);
        assertTrue(io.acquired);
        assertTrue(io.success, io.status + "; actual=" + io.actual);
    }

    @Test
    void convertsRightOfTagToDriverStationRightOnBlueAndMirrorsOnRed() {
        TargetMeasurement right = new TargetMeasurement(26, 0.2, kHopperTagStandOffMeters, 10);
        Pose2d blue = HopperTagAlignmentCommand.fieldPoseFromTag(right, GOAL, GOAL);
        assertEquals(kHopperTagCenterYMeters - 0.2, blue.getY(), 1e-9);
        Pose2d redGoal = new Pose2d(13.1, 4.0346376 - kHopperDriverStationLeftOffsetMeters, Rotation2d.kZero);
        Pose2d red = HopperTagAlignmentCommand.fieldPoseFromTag(right, redGoal, redGoal);
        assertEquals(4.0346376 + 0.2, red.getY(), 1e-9);
    }

    @Test
    void interruptionClearsActiveTrackingAndStopsBothDirections() {
        FakeDrive io = new FakeDrive(CAMERA_GOAL);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        command.end(true);
        assertFalse(io.active);
        assertFalse(io.success);
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        assertEquals(0.0, io.commanded.omegaRadiansPerSecond);
    }

    @Test
    void restoresRobotCenterAndSquareHeadingWithAnOffCenterCamera() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        io.cameraOffsetRobot = new Translation2d(-0.20, 0.08);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual + "; image=" + io.imageAngleDegrees());
        assertEquals(CAMERA_GOAL.getY(), io.actual.getY(), kHopperAlignLateralToleranceMeters);
        assertEquals(CAMERA_GOAL.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
    }

    @Test
    void correctsThirtyCentimeterStartingOffsetToActualTagCenter() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX(), GOAL.getY() + 0.30, GOAL.getRotation()));
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status);
        assertTrue(io.moved);
        assertEquals(TAG_REFERENCE.getY(), io.actual.getY(), 0.012);
        assertEquals(0.0, io.imageAngleDegrees(), 1.5);
        assertEquals(io.actual.getY(), io.odometry.getY(), 1e-9);
    }

    @Test
    void correctsReturnFromBothSidesDespiteOngoingRightwardDrift() {
        for (double offset : new double[] {-0.30, 0.30}) {
            FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX() - 0.865, CAMERA_GOAL.getY() + offset,
                CAMERA_GOAL.getRotation()));
            // A biased wheel/traction response is invisible to encoder odometry.
            io.rightwardDriftPerMeter = 0.14;
            HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond);
            command.initialize();
            run(command, io);
            assertTrue(io.success, io.status + "; start offset=" + offset + "; actual=" + io.actual);
            assertEquals(CAMERA_GOAL.getY(), io.actual.getY(), kHopperAlignToleranceMeters);
            assertTrue(io.peakReverseSpeed <= kHopperApproachMaxSpeedMetersPerSecond);
            assertTrue(io.minimumOutward >= kHopperTagStandOffMeters - 0.005,
                "Correction must not cross the hopper stand-off");
        }
    }

    @Test
    void pullsForwardBeforeTurningToCorrectAnOffsetBesideTheHub() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.3, CAMERA_GOAL.getRotation()));
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        boolean droveForward = false;
        for (int step = 0; step < 200 && CAMERA_GOAL.getX() - io.actual.getX() < 0.23; step++) {
            command.execute();
            assertTrue(io.commanded.vxMetersPerSecond >= 0.0, "First movement must be away from hub");
            assertEquals(0.0, io.commanded.omegaRadiansPerSecond, 1e-9,
                "Rear hopper must clear the hub before turning");
            droveForward |= io.commanded.vxMetersPerSecond > 0.0;
            io.step();
        }
        assertTrue(droveForward);
        command.end(true);
    }

    @Test
    void brakesForMeasuredReverseMomentumBeforeHopperStandOff() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX() - 0.4, CAMERA_GOAL.getY(), CAMERA_GOAL.getRotation()));
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        for (int step = 0; step < 20 && !io.acquired; step++) {
            command.execute();
            io.step();
        }
        assertTrue(io.acquired);
        io.actual = new Pose2d(CAMERA_GOAL.getX() - 0.06, CAMERA_GOAL.getY(), CAMERA_GOAL.getRotation());
        io.odometry = io.actual;
        io.measured = new ChassisSpeeds(-0.5, 0.0, 0.0);
        io.now += 0.04;
        command.execute();
        assertEquals(0.0, io.commanded.vxMetersPerSecond, 1e-9);
        assertTrue(io.status.contains("braking"));
        command.end(true);
    }

    @Test
    void stopsMovingWhenVisionBecomesStaleDuringReturn() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX() - 0.8, CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        for (int step = 0; step < 30; step++) {
            command.execute();
            io.step();
        }
        assertTrue(io.acquired);
        io.visible = false;
        for (int step = 0; step < 20; step++) {
            command.execute();
            io.step();
        }
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        assertEquals(0.0, io.commanded.omegaRadiansPerSecond);
        assertTrue(io.status.contains("waiting for both hopper tags"));
        command.end(true);
    }

    @Test
    void pairedStartupReferenceCorrectsPositionEvenWithPersistentFieldPoseBias() {
        FakeDrive io = new FakeDrive(GOAL);
        io.visionLateralBiasMeters = 0.18;
        var reference = captureStartView(io, GOAL, 26);
        io.actual = new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.24, CAMERA_GOAL.getRotation());
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual + "; image=" + io.imageAngleDegrees());
        assertTrue(io.moved);
        assertEquals(CAMERA_GOAL.getY(), io.actual.getY(), 0.015);
        assertEquals(0.0, io.imageAngleDegrees(), 1.5);
        command.end(false);
    }

    @Test
    void aTurnThatCentersTheImageDoesNotReplacePhysicalRecentering() {
        double yaw = Math.atan2(0.20, kHopperTagStandOffMeters);
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.20,
            CAMERA_GOAL.getRotation().plus(Rotation2d.fromRadians(yaw))));
        assertEquals(0.0, io.imageAngleDegrees(), 1e-9);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual);
        assertEquals(CAMERA_GOAL.getY(), io.actual.getY(), 0.015);
        assertEquals(0.0, io.imageAngleDegrees(), 1.5);
        command.end(false);
    }

    @Test
    void fieldLocalizationFromOtherTagsCannotConfirmTag26ImageCenter() {
        FakeDrive io = new FakeDrive(CAMERA_GOAL);
        io.imageAvailable = false;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertFalse(io.acquired, "Return correction requires both hopper tags, not other mapped tags");
        assertFalse(io.success, "Only a fresh image observation of the requested tag can confirm centering");
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        command.end(false);
    }

    @Test
    void returnRestoresTheRecordedOffCenterViewFromEitherSide() {
        for (double side : new double[] {-0.30, 0.30}) {
            FakeDrive io = new FakeDrive(GOAL);
            io.cameraOffsetRobot = new Translation2d(-0.20, 0.08);
            io.cameraYawOffsetRadians = Math.toRadians(5.0);
            var reference = captureStartView(io, GOAL, 26);
            assertTrue(Math.abs(reference.imageHorizontalDegrees()) > 3.0,
                "The saved view deliberately has the tag off image center");
            io.actual = new Pose2d(GOAL.getX() - 0.865, GOAL.getY() + side, GOAL.getRotation());
            io.odometry = GOAL;
            io.moved = false;
            HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
            command.initialize();
            StringBuilder trace = new StringBuilder();
            for (int step = 0; step < 930 && !command.isFinished(); step++) {
                command.execute();
                if (step % 50 == 0) {
                    trace.append("\n").append(io.now).append(" actual=").append(io.actual)
                        .append(" error=").append(io.odometry.relativeTo(GOAL))
                        .append(" vx=").append(io.commanded.vxMetersPerSecond)
                        .append(" w=").append(io.commanded.omegaRadiansPerSecond).append(" ").append(io.status);
                }
                io.step();
            }
            assertTrue(io.success, io.status + "; actual=" + io.actual + "; image=" + io.imageAngleDegrees() + trace);
            assertEquals(GOAL.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
            assertEquals(GOAL.getY(), io.actual.getY(), kHopperAlignToleranceMeters);
            assertEquals(reference.imageHorizontalDegrees(), io.imageAngleDegrees(), 1.5);
            assertTrue(io.moved);
            assertTrue(io.peakReverseSpeed <= kHopperApproachMaxSpeedMetersPerSecond);
            command.end(false);
        }
    }

    @Test
    void followingAlignmentKeepsTheSavedViewInsteadOfCenteringIt() {
        FakeDrive io = new FakeDrive(GOAL);
        io.cameraOffsetRobot = new Translation2d(-0.20, 0.08);
        var reference = captureStartView(io, GOAL, 26);
        io.moved = false;
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status);
        assertFalse(io.moved, "Already matching the saved startup view; do not aim for image zero or a fixed offset");
        assertEquals(reference.imageHorizontalDegrees(), io.imageAngleDegrees(), 1e-9);
        command.end(false);
    }

    @Test
    void startupViewCannotBeConfirmedUsingOnlyOtherTags() {
        FakeDrive io = new FakeDrive(GOAL);
        var reference = captureStartView(io, GOAL, 26);
        io.imageAvailable = false;
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
        command.initialize();
        run(command, io);
        assertFalse(io.success);
        assertTrue(io.status.contains("no fresh consistent position for hopper tags 25 + 26"));
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        command.end(false);
    }

    @Test
    void matchingTheSavedAngleStillRequiresRestoringTheCameraDistance() {
        FakeDrive io = new FakeDrive(GOAL);
        var reference = captureStartView(io, GOAL, 26);
        io.actual = new Pose2d(GOAL.getX() - 0.5, GOAL.getY(), GOAL.getRotation());
        io.odometry = GOAL;
        io.moved = false;
        assertEquals(reference.imageHorizontalDegrees(), io.imageAngleDegrees(), 1e-9);
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status);
        assertTrue(io.moved, "Same tag angle from farther away must not count as restoring the view");
        assertEquals(GOAL.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
        command.end(false);
    }

    @Test
    void recordedViewReturnsToThePhysicalStartDespiteFixedFieldPoseBias() {
        FakeDrive io = new FakeDrive(GOAL);
        io.cameraOffsetRobot = new Translation2d(-0.20, 0.08);
        io.visionLateralBiasMeters = 0.18;
        io.visionOutwardBiasMeters = 0.08;
        var reference = captureStartView(io, GOAL, 26);
        io.actual = new Pose2d(GOAL.getX() - 0.7, GOAL.getY() - 0.25, GOAL.getRotation());
        io.odometry = GOAL;
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual + "; image=" + io.imageAngleDegrees());
        assertEquals(GOAL.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
        assertEquals(GOAL.getY(), io.actual.getY(), kHopperAlignToleranceMeters);
        assertEquals(reference.imageHorizontalDegrees(), io.imageAngleDegrees(), 1.5);
        command.end(false);
    }

    @Test
    void savedPairReferenceCancelsFixedIndependentHeadingBias() {
        FakeDrive io = new FakeDrive(GOAL);
        io.independentHeadingBiasDegrees = 15;
        var reference = captureStartView(io, GOAL, 26);
        io.actual = new Pose2d(GOAL.getX() - 0.7, GOAL.getY() - 0.2, GOAL.getRotation());
        io.odometry = GOAL;
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, reference);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual);
        assertEquals(0.0, MathUtil.angleModulus(io.actual.getRotation().minus(GOAL.getRotation()).getRadians()),
            kHopperAlignHeadingToleranceRadians);
        assertEquals(GOAL.getY(), io.actual.getY(), kHopperAlignToleranceMeters);
        command.end(false);
    }

    static frc.robot.vision.HopperStartView.Reference captureStartView(FakeDrive io, Pose2d pose, int tagId) {
        var view = new frc.robot.vision.HopperStartView();
        var capture = new CaptureHopperStartViewCommand(io, io, view, () -> pose,
            () -> tagId, () -> io.now, () -> io.enabled);
        capture.initialize();
        for (int step = 0; step < 50 && !capture.isFinished(); step++) {
            capture.execute();
            io.step();
        }
        capture.end(false);
        return view.getReference().orElseThrow();
    }

    private static HopperTagAlignmentCommand command(FakeDrive io) {
        return new HopperTagAlignmentCommand(io, io, GOAL, 26, () -> io.now, () -> io.enabled);
    }

    private static void run(HopperTagAlignmentCommand command, FakeDrive io) {
        int maxSteps = (int) Math.ceil((kHopperReturnAlignTimeoutSeconds + 0.5) / 0.02);
        for (int step = 0; step < maxSteps && !command.isFinished(); step++) {
            command.execute();
            io.step();
        }
        assertTrue(command.isFinished(), "Command did not finish within bounded timeout");
    }

    /** Wheel/gyro odometry starts wrong; camera reports the actual pose with a rear-facing FOV. */
    static class FakeDrive implements HopperTagAlignmentCommand.IO, Subsystem {
        Pose2d actual;
        Translation2d cameraOffsetRobot = Translation2d.kZero;
        Pose2d odometry = GOAL;
        ChassisSpeeds commanded = new ChassisSpeeds();
        ChassisSpeeds measured = new ChassisSpeeds();
        double now = 10.0;
        double lastFrame = 0.0;
        boolean enabled = true;
        boolean visible = true;
        boolean loseTagAfterMoving;
        boolean reliableHeading = true;
        boolean active;
        boolean moved;
        boolean success;
        boolean acquired;
        double rightwardDriftPerMeter;
        double peakReverseSpeed;
        double minimumOutward = Double.POSITIVE_INFINITY;
        double visionLateralBiasMeters;
        double visionOutwardBiasMeters;
        double independentHeadingBiasDegrees;
        double cameraYawOffsetRadians;
        boolean imageAvailable = true;
        boolean companionAvailable = true;
        boolean invalidPairGeometry;
        boolean fullView;
        String status = "";

        FakeDrive(Pose2d actual) {
            this.actual = actual;
            odometry = new Pose2d(GOAL.getTranslation(), actual.getRotation());
        }

        Translation2d cameraPosition() {
            return actual.getTranslation().plus(cameraOffsetRobot.rotateBy(actual.getRotation()));
        }

        double imageAngleDegrees() {
            Pose2d camera = new Pose2d(cameraPosition(), actual.getRotation()).relativeTo(TAG_REFERENCE);
            return Math.toDegrees(MathUtil.angleModulus(camera.getRotation().getRadians() + cameraYawOffsetRadians
                - Math.atan2(camera.getY(), kHopperTagStandOffMeters + camera.getX())));
        }

        double companionImageAngleDegrees() {
            Pose2d camera = new Pose2d(cameraPosition(), actual.getRotation()).relativeTo(TAG_REFERENCE);
            // Tag 25 shares tag 26's face, 35.56 cm toward positive field Y.
            return Math.toDegrees(MathUtil.angleModulus(camera.getRotation().getRadians() + cameraYawOffsetRadians
                - Math.atan2(camera.getY() + 0.3556, kHopperTagStandOffMeters + camera.getX())));
        }

        double verticalAngleDegrees(boolean companion) {
            if (!fullView) { return Double.NaN; }
            var photo = HopperViewAlignment.photo();
            double target = companion ? photo.companionY() : photo.mainY();
            double referenceDistance = kHopperTagStandOffMeters + cameraOffsetRobot.getX();
            double currentDistance = kHopperTagStandOffMeters
                + new Pose2d(cameraPosition(), actual.getRotation()).relativeTo(TAG_REFERENCE).getX();
            return Math.toDegrees(Math.atan(Math.tan(Math.toRadians(target)) * referenceDistance / currentDistance));
        }

        void step() {
            measured = new ChassisSpeeds(
                measured.vxMetersPerSecond + 0.25 * (commanded.vxMetersPerSecond - measured.vxMetersPerSecond), 0,
                measured.omegaRadiansPerSecond + 0.25 * (commanded.omegaRadiansPerSecond - measured.omegaRadiansPerSecond));
            Twist2d movement = new Twist2d(measured.vxMetersPerSecond * 0.02, 0, measured.omegaRadiansPerSecond * 0.02);
            actual = actual.exp(movement);
            if (Math.abs(measured.vxMetersPerSecond) > 0.01) {
                actual = new Pose2d(actual.getX(), actual.getY()
                    - rightwardDriftPerMeter * Math.abs(measured.vxMetersPerSecond) * 0.02,
                    actual.getRotation());
            }
            minimumOutward = Math.min(minimumOutward,
                kHopperTagStandOffMeters + actual.relativeTo(CAMERA_GOAL).getX());
            odometry = odometry.exp(movement);
            now += 0.02;
        }

        public Pose2d getPose() { return odometry; }
        public ChassisSpeeds getRobotRelativeSpeeds() { return measured; }
        public void resetOdometry(Pose2d pose) { odometry = pose; }
        public void driveRobotRelative(ChassisSpeeds speeds) {
            commanded = speeds;
            peakReverseSpeed = Math.max(peakReverseSpeed, -speeds.vxMetersPerSecond);
            moved |= Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.omegaRadiansPerSecond) > 0.001;
        }
        public void stopMotors() { commanded = new ChassisSpeeds(); }
        public void beginHopperTagAlignment(int id) { active = true; success = false; acquired = false; }
        public void onHopperPoseAcquired() { acquired = true; }
        public void endHopperTagAlignment() { active = false; }
        public void fuseHopperTagPose(Pose2d pose, double timestamp) { odometry = pose; }
        public void reportHopperTagAlignment(String value, boolean aligned, double right) { status = value; success = aligned; }

        public Optional<TargetMeasurement> getNewHopperTagPose(int tagId) {
            if (!visible || (loseTagAfterMoving && moved) || now - lastFrame < 0.039) {
                return Optional.empty();
            }
            lastFrame = now;
            double imageAngle = imageAngleDegrees();
            double companionAngle = invalidPairGeometry ? imageAngle - 0.1 : companionImageAngleDegrees();
            boolean pair = imageAvailable && companionAvailable
                && Math.abs(imageAngle) < 70 && Math.abs(companionAngle) < 70;
            Pose2d relative = new Pose2d(actual.getX(), actual.getY() + visionLateralBiasMeters,
                actual.getRotation()).relativeTo(TAG_REFERENCE);
            Pose2d cameraRelative = new Pose2d(cameraPosition().plus(new Translation2d(0, visionLateralBiasMeters)),
                actual.getRotation()).relativeTo(TAG_REFERENCE);
            return Optional.of(new TargetMeasurement(tagId, relative.getY(),
                kHopperTagStandOffMeters + relative.getX() + visionOutwardBiasMeters, now - 0.02,
                reliableHeading ? actual.getRotation().getDegrees() + independentHeadingBiasDegrees : Double.NaN,
                cameraRelative.getY(), kHopperTagStandOffMeters + cameraRelative.getX() + visionOutwardBiasMeters,
                imageAvailable && Math.abs(imageAngle) < 70.0 ? imageAngle : Double.NaN,
                companionAvailable && Math.abs(companionAngle) < 70.0 ? companionAngle : Double.NaN, pair,
                verticalAngleDegrees(false), verticalAngleDegrees(true)));
        }
    }
}
