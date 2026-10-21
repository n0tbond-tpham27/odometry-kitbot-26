package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.OdometryConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.vision.Limelight.TargetMeasurement;

class HopperTagAlignmentCommandTest {
    private static final Pose2d GOAL = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));

    @Test
    void physicallyMovesLeftToTag26EvenWhenOdometryClaimsAlreadyCentered() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX(), GOAL.getY() - 0.25, GOAL.getRotation()));
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status + "; actual=" + io.actual + "; camera=" + io.cameraPosition());
        assertTrue(io.moved, "Must physically move, not only reset odometry");
        assertTrue(io.actual.getY() > GOAL.getY() - 0.04, "Must move left from Driver Station view");
        assertTrue(io.actual.getTranslation().getDistance(GOAL.getTranslation()) <= kHopperAlignToleranceMeters + 0.005);
        command.end(false);
        assertFalse(io.active);
        assertEquals(0.0, io.commanded.vxMetersPerSecond);
        assertEquals(0.0, io.commanded.omegaRadiansPerSecond);
    }

    @Test
    void seedsGyroFieldHeadingFromTagInsteadOfAssumingRobotWasPlacedAtCorrectAngle() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX(), GOAL.getY() - 0.2, GOAL.getRotation()));
        io.odometry = new Pose2d(GOAL.getTranslation(), Rotation2d.kZero);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status);
        assertTrue(io.actual.getTranslation().getDistance(GOAL.getTranslation()) < 0.04);
    }

    @Test
    void cannotDeclareSuccessFromOdometryWithoutFinalTagConfirmation() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX(), GOAL.getY() - 0.2, GOAL.getRotation()));
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
        FakeDrive io = new FakeDrive(GOAL);
        io.visible = false;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertFalse(io.success);
        assertFalse(io.moved);
        assertTrue(io.status.contains("no fresh consistent tag 26"));

        io.enabled = false;
        command.initialize();
        command.execute();
        assertTrue(command.isFinished());
        assertTrue(io.status.contains("vision disabled"));
    }

    @Test
    void refusesToMoveWithoutReliableInitialFieldHeading() {
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX(), GOAL.getY() - 0.2, GOAL.getRotation()));
        io.reliableHeading = false;
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertFalse(io.moved);
        assertFalse(io.success);
        assertTrue(io.status.contains("no reliable tag heading"));
    }

    @Test
    void convertsRightOfTagToDriverStationRightOnBlueAndMirrorsOnRed() {
        TargetMeasurement right = new TargetMeasurement(26, 0.2, kHopperTagStandOffMeters, 10);
        Pose2d blue = HopperTagAlignmentCommand.fieldPoseFromTag(right, GOAL, GOAL);
        assertEquals(GOAL.getY() - 0.2, blue.getY(), 1e-9);
        Pose2d redGoal = new Pose2d(13.1, 4.0346376, Rotation2d.kZero);
        Pose2d red = HopperTagAlignmentCommand.fieldPoseFromTag(right, redGoal, redGoal);
        assertEquals(redGoal.getY() + 0.2, red.getY(), 1e-9);
    }

    @Test
    void interruptionClearsActiveTrackingAndStopsBothDirections() {
        FakeDrive io = new FakeDrive(GOAL);
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
        FakeDrive io = new FakeDrive(new Pose2d(GOAL.getX(), GOAL.getY() - 0.2, GOAL.getRotation()));
        io.cameraOffsetRobot = new Translation2d(-0.20, 0.08);
        HopperTagAlignmentCommand command = command(io);
        command.initialize();
        run(command, io);
        assertTrue(io.success, io.status);
        assertEquals(GOAL.getY(), io.cameraPosition().getY(), kHopperAlignToleranceMeters);
        assertEquals(GOAL.getY() + 0.08, io.actual.getY(), kHopperAlignToleranceMeters);
        assertEquals(GOAL.getX(), io.actual.getX(), kHopperAlignToleranceMeters);
    }

    private static HopperTagAlignmentCommand command(FakeDrive io) {
        return new HopperTagAlignmentCommand(io, io, GOAL, 26, () -> io.now, () -> io.enabled);
    }

    private static void run(HopperTagAlignmentCommand command, FakeDrive io) {
        for (int step = 0; step < 550 && !command.isFinished(); step++) {
            command.execute();
            io.step();
        }
        assertTrue(command.isFinished(), "Command did not finish within bounded timeout");
    }

    /** Wheel/gyro odometry starts wrong; camera reports the actual pose with a rear-facing FOV. */
    private static class FakeDrive implements HopperTagAlignmentCommand.IO, Subsystem {
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
        String status = "";

        FakeDrive(Pose2d actual) { this.actual = actual; }

        Translation2d cameraPosition() {
            return actual.getTranslation().plus(cameraOffsetRobot.rotateBy(actual.getRotation()));
        }

        void step() {
            measured = new ChassisSpeeds(
                measured.vxMetersPerSecond + 0.25 * (commanded.vxMetersPerSecond - measured.vxMetersPerSecond), 0,
                measured.omegaRadiansPerSecond + 0.25 * (commanded.omegaRadiansPerSecond - measured.omegaRadiansPerSecond));
            Twist2d movement = new Twist2d(measured.vxMetersPerSecond * 0.02, 0, measured.omegaRadiansPerSecond * 0.02);
            actual = actual.exp(movement);
            odometry = odometry.exp(movement);
            now += 0.02;
        }

        public Pose2d getPose() { return odometry; }
        public ChassisSpeeds getRobotRelativeSpeeds() { return measured; }
        public void resetOdometry(Pose2d pose) { odometry = pose; }
        public void driveRobotRelative(ChassisSpeeds speeds) {
            commanded = speeds;
            moved |= Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.omegaRadiansPerSecond) > 0.001;
        }
        public void stopMotors() { commanded = new ChassisSpeeds(); }
        public void beginHopperTagAlignment(int id) { active = true; success = false; }
        public void endHopperTagAlignment() { active = false; }
        public void fuseHopperTagPose(Pose2d pose, double timestamp) { odometry = pose; }
        public void reportHopperTagAlignment(String value, boolean aligned, double right) { status = value; success = aligned; }

        public Optional<TargetMeasurement> getNewHopperTagPose(int tagId) {
            if (!visible || (loseTagAfterMoving && moved) || now - lastFrame < 0.039) {
                return Optional.empty();
            }
            // The camera can temporarily lose the tag during the lateral correction's turn.
            double turnFromGoal = actual.getRotation().minus(GOAL.getRotation()).getRadians();
            if (Math.abs(turnFromGoal) > Math.toRadians(35)) {
                return Optional.empty();
            }
            lastFrame = now;
            Pose2d relative = actual.relativeTo(GOAL);
            Pose2d cameraRelative = new Pose2d(cameraPosition(), actual.getRotation()).relativeTo(GOAL);
            return Optional.of(new TargetMeasurement(tagId, relative.getY(),
                kHopperTagStandOffMeters + relative.getX(), now - 0.02,
                reliableHeading ? actual.getRotation().getDegrees() : Double.NaN,
                cameraRelative.getY(), kHopperTagStandOffMeters + cameraRelative.getX()));
        }
    }
}
