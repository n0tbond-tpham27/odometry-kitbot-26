package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.OdometryConstants.*;
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

class HopperTagAlignmentCommandTest {
    private static final Pose2d GOAL = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));
    private static final Pose2d CAMERA_GOAL = new Pose2d(kHopperCenterXMeters, kHopperTagCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));
    private static final Pose2d TAG_REFERENCE = new Pose2d(kHopperCenterXMeters, kHopperTagCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));

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
    void seedsGyroFieldHeadingFromTagInsteadOfAssumingRobotWasPlacedAtCorrectAngle() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        io.odometry = new Pose2d(CAMERA_GOAL.getTranslation(), Rotation2d.kZero);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual + "; image=" + io.imageAngleDegrees());
        assertTrue(io.actual.getTranslation().getDistance(CAMERA_GOAL.getTranslation()) < 0.04);
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
        assertTrue(io.status.contains("no fresh consistent position for hopper tag 26"));

        io.enabled = false;
        command.initialize();
        command.execute();
        assertTrue(command.isFinished());
        assertTrue(io.status.contains("vision disabled"));
    }

    @Test
    void usesSeededGyroWhenIndependentTagHeadingIsUnavailable() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        io.reliableHeading = false;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.moved);
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
    void centersCameraInsteadOfRobotWhenCameraIsMountedOffCenter() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.2, CAMERA_GOAL.getRotation()));
        io.cameraOffsetRobot = new Translation2d(-0.20, 0.08);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual + "; image=" + io.imageAngleDegrees());
        assertEquals(CAMERA_GOAL.getY(), io.cameraPosition().getY(), kHopperAlignToleranceMeters);
        assertEquals(CAMERA_GOAL.getY() + 0.08, io.actual.getY(), kHopperAlignToleranceMeters);
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
        assertTrue(io.status.contains("waiting for fresh"));
        command.end(true);
    }

    @Test
    void imageCenterCorrectsPhysicalPositionEvenWithPersistentFieldPoseBias() {
        FakeDrive io = new FakeDrive(new Pose2d(CAMERA_GOAL.getX(), CAMERA_GOAL.getY() - 0.24,
            CAMERA_GOAL.getRotation()));
        io.visionLateralBiasMeters = 0.18;
        HopperTagAlignmentCommand command = command(io);
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
        assertTrue(io.acquired, "Other tags may still provide field positioning");
        assertFalse(io.success, "Only a fresh image observation of the requested tag can confirm centering");
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        command.end(false);
    }

    @Test
    void autonomousReturnPhysicallyMovesFiftyCentimetersLeftAndKeepsASquareHeading() {
        for (double forwardClearance : new double[] {0.0, 0.865}) {
            for (double startingOffset : new double[] {-0.30, 0.0, 0.30}) {
                FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX() - forwardClearance,
                    GOAL.getY() + startingOffset, GOAL.getRotation()));
                HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
                    () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, kHopperReturnLeftOffsetMeters);
                command.initialize();
                run(command, io);
                assertTrue(io.success, io.status + "; actual=" + io.actual);
                assertEquals(GOAL.getY() + 0.50, io.actual.getY(), kHopperAlignToleranceMeters);
                assertEquals(GOAL.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
                assertTrue(Math.abs(MathUtil.angleModulus(io.actual.getRotation().minus(GOAL.getRotation()).getRadians()))
                    <= kHopperAlignHeadingToleranceRadians);
                assertTrue(io.moved);
                assertTrue(Math.abs(io.imageAngleDegrees()) > 35.0,
                    "The large offset may put tag 26 outside the image; mapped-tag field vision confirms the target");
                assertTrue(io.peakReverseSpeed <= kHopperApproachMaxSpeedMetersPerSecond);
                command.end(false);
            }
        }
    }

    @Test
    void followingAlignmentDoesNotUndoTheLeftReturnOffset() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX(), GOAL.getY() + 0.50, GOAL.getRotation()));
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, kHopperReturnLeftOffsetMeters);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual);
        assertFalse(io.moved, "Already at the 50 cm return offset; do not recenter to zero");
        assertEquals(GOAL.getY() + 0.50, io.actual.getY(), 1e-9);
        command.end(false);
    }

    @Test
    void leftReturnOffsetMirrorsForTheRedDriverStation() {
        Pose2d redGoal = new Pose2d(13.1, 4.0346376, Rotation2d.kZero);
        FakeDrive io = new FakeDrive(new Pose2d(redGoal.getX() + 0.865, redGoal.getY(), redGoal.getRotation())) {
            @Override
            public Optional<TargetMeasurement> getNewHopperTagPose(int tagId) {
                if (now - lastFrame < 0.039) {
                    return Optional.empty();
                }
                lastFrame = now;
                Pose2d relative = actual.relativeTo(redGoal);
                return Optional.of(new TargetMeasurement(tagId, relative.getY(),
                    kHopperTagStandOffMeters + relative.getX(), now - 0.02,
                    actual.getRotation().getDegrees()));
            }
        };
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, redGoal, 10,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, kHopperReturnLeftOffsetMeters);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual);
        assertEquals(redGoal.getY() - 0.50, io.actual.getY(), kHopperAlignToleranceMeters);
        assertEquals(redGoal.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
        command.end(false);
    }

    @Test
    void returnOffsetCannotCompleteAfterLosingAllVision() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX() - 0.865, GOAL.getY(), GOAL.getRotation()));
        io.loseTagAfterMoving = true;
        HopperTagAlignmentCommand command = new HopperTagAlignmentCommand(io, io, GOAL, 26,
            () -> io.now, () -> io.enabled, kHopperApproachMaxSpeedMetersPerSecond, kHopperReturnLeftOffsetMeters);
        command.initialize();
        run(command, io);
        assertTrue(io.moved);
        assertFalse(io.success);
        assertTrue(io.status.contains("timed out"));
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        command.end(false);
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
        boolean imageAvailable = true;
        String status = "";

        FakeDrive(Pose2d actual) { this.actual = actual; }

        Translation2d cameraPosition() {
            return actual.getTranslation().plus(cameraOffsetRobot.rotateBy(actual.getRotation()));
        }

        double imageAngleDegrees() {
            Pose2d camera = new Pose2d(cameraPosition(), actual.getRotation()).relativeTo(TAG_REFERENCE);
            return Math.toDegrees(MathUtil.angleModulus(camera.getRotation().getRadians()
                - Math.atan2(camera.getY(), kHopperTagStandOffMeters + camera.getX())));
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
            // Other mapped tags can keep field localization alive if tag 26 leaves the image.
            double imageAngle = imageAngleDegrees();
            Pose2d relative = new Pose2d(actual.getX(), actual.getY() + visionLateralBiasMeters,
                actual.getRotation()).relativeTo(TAG_REFERENCE);
            Pose2d cameraRelative = new Pose2d(cameraPosition().plus(new Translation2d(0, visionLateralBiasMeters)),
                actual.getRotation()).relativeTo(TAG_REFERENCE);
            return Optional.of(new TargetMeasurement(tagId, relative.getY(),
                kHopperTagStandOffMeters + relative.getX(), now - 0.02,
                reliableHeading ? actual.getRotation().getDegrees() : Double.NaN,
                cameraRelative.getY(), kHopperTagStandOffMeters + cameraRelative.getX(),
                imageAvailable && Math.abs(imageAngle) <= 35.0 ? imageAngle : Double.NaN));
        }
    }
}
