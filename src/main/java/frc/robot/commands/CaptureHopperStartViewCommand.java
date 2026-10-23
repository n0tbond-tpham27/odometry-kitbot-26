package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.vision.HopperStartView;
import frc.robot.vision.Limelight;

/** Takes the reference before shooting or wiggling, without changing the known starting pose. */
public final class CaptureHopperStartViewCommand extends Command {
    private final HopperTagAlignmentCommand.IO m_io;
    private final HopperStartView m_view;
    private final Supplier<Pose2d> m_startingPose;
    private final IntSupplier m_tagId;
    private final DoubleSupplier m_clock;
    private final BooleanSupplier m_enabled;
    private double m_started;
    private int m_selectedTag;

    public CaptureHopperStartViewCommand(HopperTagAlignmentCommand.IO io, Subsystem requirement,
            HopperStartView view, Supplier<Pose2d> startingPose, IntSupplier tagId,
            DoubleSupplier clock, BooleanSupplier visionEnabled) {
        m_io = io;
        m_view = view;
        m_startingPose = startingPose;
        m_tagId = tagId;
        m_clock = clock;
        m_enabled = visionEnabled;
        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        m_started = m_clock.getAsDouble();
        m_selectedTag = m_tagId.getAsInt();
        m_view.reset(m_selectedTag, m_startingPose.get(), m_started);
        m_io.stopMotors();
        m_io.beginHopperTagAlignment(m_selectedTag);
        SmartDashboard.putBoolean("Vision/Start View Saved", false);
        SmartDashboard.putString("Vision/Start View Status", "capturing stationary hopper tags "
            + Limelight.companionTagId(m_selectedTag) + " + " + m_selectedTag);
    }

    @Override
    public void execute() {
        m_io.driveRobotRelative(new ChassisSpeeds());
        m_view.observeMotion(m_io.getRobotRelativeSpeeds(), m_clock.getAsDouble());
        if (m_enabled.getAsBoolean()) {
            m_io.getNewHopperTagPose(m_selectedTag).ifPresent(sample ->
                m_view.add(sample, m_io.getRobotRelativeSpeeds(), m_clock.getAsDouble()));
        }
    }

    @Override
    public boolean isFinished() {
        return m_view.getReference().isPresent() || !m_enabled.getAsBoolean()
            || m_clock.getAsDouble() - m_started >= kHopperStartViewCaptureTimeoutSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        m_io.stopMotors();
        m_io.endHopperTagAlignment();
        if (interrupted) {
            m_view.reset(m_selectedTag, m_startingPose.get(), m_clock.getAsDouble());
        }
        boolean saved = m_view.getReference().isPresent();
        SmartDashboard.putBoolean("Vision/Start View Saved", saved);
        SmartDashboard.putString("Vision/Start View Status", saved ? "saved for this autonomous run"
            : interrupted ? "capture cancelled" : "no matching live view; return uses hub photo estimate and gyro");
        m_view.getReference().ifPresent(reference -> {
            SmartDashboard.putNumber("Vision/Start View TX (deg)", reference.imageHorizontalDegrees());
            SmartDashboard.putNumber("Vision/Start View Companion TX (deg)", reference.companionImageHorizontalDegrees());
            SmartDashboard.putNumber("Vision/Start View TY (deg)", reference.imageVerticalDegrees());
            SmartDashboard.putNumber("Vision/Start View Companion TY (deg)", reference.companionImageVerticalDegrees());
            SmartDashboard.putNumber("Vision/Start View Camera Range (m)", reference.cameraOutwardMeters());
            SmartDashboard.putNumber("Vision/Start View Robot Range (m)", reference.robotOutwardMeters());
        });
    }
}
