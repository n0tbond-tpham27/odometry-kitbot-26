package frc.robot.commands;

import static frc.robot.Constants.OdometryConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.vision.HopperStartView;

class CaptureHopperStartViewCommandTest {
    private static final Pose2d START = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void missingStartupTagReleasesTheAutoWithoutDrivingOrReusingAnOldReference() {
        var drive = new HopperTagAlignmentCommandTest.FakeDrive(START);
        var view = new HopperStartView();
        drive.visible = false;
        AtomicBoolean nextStep = new AtomicBoolean();
        Command auto = capture(drive, view).andThen(Commands.runOnce(() -> nextStep.set(true)));
        auto.initialize();
        for (int i = 0; i < 50 && !auto.isFinished(); i++) {
            auto.execute();
            drive.step();
        }
        assertTrue(auto.isFinished());
        auto.end(false);
        assertTrue(nextStep.get(), "Missing startup vision must not hold the entire auto");
        assertTrue(view.getReference().isEmpty());
        assertFalse(drive.moved);
        assertFalse(drive.active);
        assertEquals(START, drive.odometry, "Capture must not change the known square starting pose");
    }

    @Test
    void cancellingCaptureClearsItsReferenceAndReleasesVisionOwnership() {
        var drive = new HopperTagAlignmentCommandTest.FakeDrive(START);
        var view = new HopperStartView();
        Command command = capture(drive, view);
        command.initialize();
        command.execute();
        command.end(true);
        assertTrue(view.getReference().isEmpty());
        assertFalse(drive.active);
        assertEquals(0.0, drive.commanded.vxMetersPerSecond);
    }

    private static Command capture(HopperTagAlignmentCommandTest.FakeDrive drive, HopperStartView view) {
        return new CaptureHopperStartViewCommand(drive, drive, view, () -> START,
            () -> 26, () -> drive.now, () -> drive.enabled);
    }
}
