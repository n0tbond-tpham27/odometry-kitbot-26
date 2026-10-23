package frc.robot.vision;

import static frc.robot.Constants.OdometryConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.vision.Limelight.TargetMeasurement;

class HopperStartViewTest {
    private static final Pose2d START = new Pose2d(kHopperCenterXMeters, kHopperCenterYMeters,
        Rotation2d.fromDegrees(kHopperCenterHeadingDegrees));
    private static final ChassisSpeeds STOPPED = new ChassisSpeeds();

    @Test
    void averagesFreshStationaryCapturesAndFreezesTheViewBeforeWiggling() {
        var view = new HopperStartView();
        view.reset(26, START, 10);
        view.add(sample(26, 10.02, 0.40, 6.0), STOPPED, 10.04);
        view.add(sample(26, 10.06, 0.41, 6.2), STOPPED, 10.08);
        assertTrue(view.getReference().isEmpty());
        view.add(sample(26, 10.10, 0.39, 5.8), STOPPED, 10.12);
        var saved = view.getReference().orElseThrow();
        assertEquals(6.0, saved.imageHorizontalDegrees(), 1e-9);
        assertEquals(-19.0, saved.companionImageHorizontalDegrees(), 1e-9);
        assertEquals(START.getX(), saved.pairedFieldPose().getX(), 1e-9);
        assertEquals(START.getY(), saved.pairedFieldPose().getY(), 1e-9);
        assertEquals(0.40, saved.cameraOutwardMeters(), 1e-9);
        assertEquals(kHopperTagStandOffMeters, saved.robotOutwardMeters(), 1e-9);
        assertEquals(START, saved.startingPose());
        view.add(sample(26, 11.0, 0.60, 20), new ChassisSpeeds(0.12, 0, 0), 11.02);
        assertEquals(saved, view.getReference().orElseThrow(), "Shooting wiggles must not update the startup reference");
    }

    @Test
    void productionCaptureRefinesThePhotoInsteadOfLearningAnUnrelatedStartingView() {
        var view = new HopperStartView(true);
        view.reset(26, START, 10);
        for (int i = 1; i <= 4; i++) {
            view.add(sample(26, 10 + i * 0.04, 0.4, 0), STOPPED, 10.02 + i * 0.04);
        }
        assertTrue(view.getReference().isEmpty());
        var photo = HopperViewAlignment.photo();
        for (int i = 1; i <= 3; i++) {
            double capture = 10.20 + i * 0.04;
            view.add(new TargetMeasurement(26, 0, kHopperTagStandOffMeters, capture,
                180, 0, 1.05, photo.mainX() + 0.4, photo.companionX() - 0.2, true,
                photo.mainY() + 0.3, photo.companionY() + 0.1), STOPPED, capture + 0.02);
        }
        var reference = view.getReference().orElseThrow();
        assertEquals(photo.mainX() + 0.4, reference.imageHorizontalDegrees(), 1e-9);
        assertEquals(photo.mainY() + 0.3, reference.imageVerticalDegrees(), 1e-9);
        assertEquals(photo.companionY() + 0.1, reference.companionImageVerticalDegrees(), 1e-9);
    }

    @Test
    void anUnpairedPoseCannotBeSavedAsThePairedStartupView() {
        var view = new HopperStartView();
        view.reset(26, START, 10);
        for (int i = 0; i < 5; i++) {
            double time = 10.02 + 0.04 * i;
            view.add(new TargetMeasurement(26, 0, kHopperTagStandOffMeters, time,
                180, 0, 0.4, 6), STOPPED, time + 0.02);
        }
        assertTrue(view.getReference().isEmpty());
    }

    @Test
    void rejectsOldRepeatedWrongTagAndInvalidImageCaptures() {
        var view = new HopperStartView();
        view.reset(26, START, 10);
        view.add(sample(26, 9.9, 0.4, 6), STOPPED, 10.02);
        view.add(sample(26, 10.6, 0.4, 6), STOPPED, 10.02);
        view.add(sample(25, 10.02, 0.4, 6), STOPPED, 10.04);
        view.add(sample(26, 10.02, 0.4, Double.NaN), STOPPED, 10.04);
        view.add(sample(26, 10.02, Double.NaN, 6), STOPPED, 10.04);
        view.add(sample(26, 10.02, 0.4, 6), STOPPED, 10.04);
        for (int i = 0; i < 5; i++) {
            view.add(sample(26, 10.02, 0.4, 6), STOPPED, 10.08);
        }
        assertTrue(view.getReference().isEmpty(), "Only one distinct valid capture was supplied");
    }

    @Test
    void movementWithoutACameraFrameClearsThePartialReference() {
        var view = new HopperStartView();
        view.reset(26, START, 10);
        view.add(sample(26, 10.02, 0.4, 6), STOPPED, 10.04);
        view.add(sample(26, 10.06, 0.4, 6), STOPPED, 10.08);
        view.observeMotion(new ChassisSpeeds(0.2, 0, 0), 10.10);
        view.add(sample(26, 10.09, 0.4, 6), STOPPED, 10.12);
        view.add(sample(26, 10.14, 0.4, 6), STOPPED, 10.16);
        assertTrue(view.getReference().isEmpty());
        view.add(sample(26, 10.18, 0.4, 6), STOPPED, 10.20);
        view.add(sample(26, 10.22, 0.4, 6), STOPPED, 10.24);
        assertTrue(view.getReference().isPresent());
    }

    @Test
    void startsANewClusterAfterAnOutlierAndClearsOldRunsOnReset() {
        var view = new HopperStartView();
        view.reset(26, START, 10);
        view.add(sample(26, 10.02, 0.4, 6), STOPPED, 10.04);
        view.add(sample(26, 10.06, 0.5, 12), STOPPED, 10.08);
        view.add(sample(26, 10.10, 0.5, 12), STOPPED, 10.12);
        assertTrue(view.getReference().isEmpty());
        view.add(sample(26, 10.14, 0.5, 12), STOPPED, 10.16);
        assertEquals(12, view.getReference().orElseThrow().imageHorizontalDegrees(), 1e-9);
        view.reset(10, new Pose2d(13.1, 4.0346376, Rotation2d.kZero), 20);
        assertTrue(view.getReference().isEmpty(), "Never reuse a previous run or alliance's camera reference");
    }

    private static TargetMeasurement sample(int tag, double capture, double cameraRange, double angle) {
        return new TargetMeasurement(tag, 0, kHopperTagStandOffMeters, capture,
            180, 0, cameraRange, angle, angle - 25, true);
    }
}
