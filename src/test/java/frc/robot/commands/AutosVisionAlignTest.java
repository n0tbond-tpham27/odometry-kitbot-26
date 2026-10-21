package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.OdometryConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

class AutosVisionAlignTest {
    private static final Pose2d GOAL = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));

    @BeforeAll
    static void startSimulationClock() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
    }

    @AfterAll
    static void resumeClock() {
        SimHooks.resumeTiming();
    }

    @Test
    void missingTagAdvancesToNextAutoStepAfterAlignmentFailure() {
        var drive = new HopperTagAlignmentCommandTest.FakeDrive(new Pose2d(GOAL.getX(), kHopperTagCenterYMeters, GOAL.getRotation()));
        drive.visible = false;
        AtomicBoolean nextStep = new AtomicBoolean();
        Command attempt = new HopperTagAlignmentCommand(drive, drive, GOAL, 26, () -> drive.now, () -> true);
        Command auto = Autos.boundedVisionAlign(attempt, drive::stopMotors)
            .andThen(Commands.runOnce(() -> nextStep.set(true)));
        run(auto, drive, 2.0);
        assertTrue(nextStep.get(), "A missing tag must not hold the entire auto");
        assertFalse(drive.success);
        assertFalse(drive.active);
        assertTrue(drive.status.contains("alignment failed"));
        assertEquals(0.0, drive.commanded.vxMetersPerSecond);
    }

    @Test
    void successfulAlignmentAlsoAdvancesToNextAutoStep() {
        var drive = new HopperTagAlignmentCommandTest.FakeDrive(new Pose2d(GOAL.getX(), kHopperTagCenterYMeters, GOAL.getRotation()));
        AtomicBoolean nextStep = new AtomicBoolean();
        Command attempt = new HopperTagAlignmentCommand(drive, drive, GOAL, 26, () -> drive.now, () -> true);
        Command auto = Autos.boundedVisionAlign(attempt, drive::stopMotors)
            .andThen(Commands.runOnce(() -> nextStep.set(true)));
        run(auto, drive, 2.0);
        assertTrue(nextStep.get());
        assertTrue(drive.success);
        assertFalse(drive.active);
    }

    @Test
    void outerTimeoutReleasesEvenAnAlignmentThatNeverFinishes() {
        AtomicBoolean interrupted = new AtomicBoolean();
        AtomicBoolean stopped = new AtomicBoolean();
        AtomicBoolean nextStep = new AtomicBoolean();
        Command neverFinishes = Commands.run(() -> {}).finallyDo(interrupted::set);
        Command auto = Autos.boundedVisionAlign(neverFinishes, () -> stopped.set(true))
            .andThen(Commands.runOnce(() -> nextStep.set(true)));
        double started = Timer.getFPGATimestamp();
        run(auto, null, kHopperAlignTimeoutSeconds + 0.1);
        assertTrue(interrupted.get());
        assertTrue(stopped.get());
        assertTrue(nextStep.get());
        assertTrue(Timer.getFPGATimestamp() - started <= kHopperAlignTimeoutSeconds + 0.1);
    }

    @Test
    void cancellationStopsAlignmentWithoutStartingTheNextStep() {
        var drive = new HopperTagAlignmentCommandTest.FakeDrive(new Pose2d(GOAL.getX(), kHopperTagCenterYMeters, GOAL.getRotation()));
        AtomicBoolean nextStep = new AtomicBoolean();
        Command attempt = new HopperTagAlignmentCommand(drive, drive, GOAL, 26, () -> drive.now, () -> true);
        Command auto = Autos.boundedVisionAlign(attempt, drive::stopMotors)
            .andThen(Commands.runOnce(() -> nextStep.set(true)));
        auto.initialize();
        auto.execute();
        auto.end(true);
        assertFalse(nextStep.get());
        assertFalse(drive.active);
        assertEquals(0.0, drive.commanded.vxMetersPerSecond);
        assertEquals(0.0, drive.commanded.omegaRadiansPerSecond);
    }

    @Test
    void resetReturnOnlyFallsBackWhenVisionNeverAcquired() {
        for (int scenario = 0; scenario < 3; scenario++) {
            var drive = new HopperTagAlignmentCommandTest.FakeDrive(scenario == 2
                ? new Pose2d(GOAL.getX() - 0.8, GOAL.getY() - 0.2, GOAL.getRotation()) : new Pose2d(GOAL.getX(), kHopperTagCenterYMeters, GOAL.getRotation()));
            drive.visible = scenario != 1;
            drive.loseTagAfterMoving = scenario == 2;
            AtomicBoolean fallback = new AtomicBoolean();
            Command attempt = new HopperTagAlignmentCommand(drive, drive, GOAL, 26, () -> drive.now, () -> true);
            Command auto = Autos.boundedVisionAlign(attempt, drive::stopMotors)
                .andThen(Autos.returnAfterVision(Commands.runOnce(() -> fallback.set(true)),
                    () -> drive.success, () -> drive.acquired, drive::stopMotors));
            run(auto, drive, kHopperAlignTimeoutSeconds + 0.2);
            assertEquals(scenario == 1, fallback.get(), "Only a return with no usable vision may use the path");
            assertEquals(scenario == 0, drive.success);
            assertEquals(scenario != 1, drive.acquired);
            assertEquals(0.0, drive.commanded.vxMetersPerSecond);
        }
    }

    private static void run(Command auto, HopperTagAlignmentCommandTest.FakeDrive drive, double maxSeconds) {
        auto.initialize();
        for (int i = 0; i < (int) Math.ceil(maxSeconds / 0.02) && !auto.isFinished(); i++) {
            auto.execute();
            if (drive != null) {
                drive.step();
            }
            SimHooks.stepTiming(0.02);
        }
        assertTrue(auto.isFinished(), "Auto failed to progress within its time budget");
        auto.end(false);
    }
}
