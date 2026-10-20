package frc.robot.vision;

import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Turn, drive onto the hopper center, then face away from it with a differential drivetrain. */
public final class HopperAlignmentController {
    private enum Phase { TURN_TO_POSITION, DRIVE_TO_POSITION, FINAL_HEADING, FINISHED }

    private final Pose2d m_target;
    private Phase m_phase = Phase.TURN_TO_POSITION;
    private double m_direction = 1.0;

    public HopperAlignmentController(Pose2d target) {
        m_target = target;
    }

    /** Choose the shorter turn, allowing the robot to back toward the hopper. */
    public void reset(Pose2d current) {
        Translation2d offset = m_target.getTranslation().minus(current.getTranslation());
        double bearingError = MathUtil.angleModulus(
            offset.getAngle().minus(current.getRotation()).getRadians());
        m_direction = Math.abs(bearingError) > Math.PI / 2.0 ? -1.0 : 1.0;
        m_phase = Phase.TURN_TO_POSITION;
    }

    /** Recalculate from the latest vision-corrected pose; no sideways drive is assumed. */
    public ChassisSpeeds calculate(Pose2d current, ChassisSpeeds measuredSpeeds) {
        Translation2d offset = m_target.getTranslation().minus(current.getTranslation());
        double distance = offset.getNorm();
        double finalHeadingError = MathUtil.angleModulus(
            m_target.getRotation().minus(current.getRotation()).getRadians());
        boolean stopped = Math.abs(measuredSpeeds.vxMetersPerSecond)
                < kDriveStraightSpeedToleranceMetersPerSecond
            && Math.abs(measuredSpeeds.omegaRadiansPerSecond) < kHopperAlignTurnSpeedToleranceRadPerSec;

        if (m_phase == Phase.FINISHED) {
            return new ChassisSpeeds();
        }
        if (distance <= kHopperAlignToleranceMeters) {
            m_phase = Phase.FINAL_HEADING;
        } else if (m_phase == Phase.FINAL_HEADING) {
            // A position correction may arrive while turning. Reacquire the center if needed.
            reset(current);
        }

        if (m_phase == Phase.FINAL_HEADING) {
            if (Math.abs(finalHeadingError) <= kHopperAlignHeadingToleranceRadians) {
                if (stopped) {
                    m_phase = Phase.FINISHED;
                }
                return new ChassisSpeeds();
            }
            return new ChassisSpeeds(0.0, 0.0, turnRate(finalHeadingError));
        }

        double travelHeading = offset.getAngle().getRadians() + (m_direction < 0.0 ? Math.PI : 0.0);
        double bearingError = MathUtil.angleModulus(travelHeading - current.getRotation().getRadians());
        if (m_phase == Phase.DRIVE_TO_POSITION
                && Math.abs(bearingError) > kHopperAlignReorientRadians) {
            reset(current);
            return new ChassisSpeeds();
        }
        if (m_phase == Phase.TURN_TO_POSITION) {
            if (Math.abs(bearingError) > kHopperAlignHeadingToleranceRadians) {
                return new ChassisSpeeds(0.0, 0.0, turnRate(bearingError));
            }
            // Let the spin settle before starting translation.
            if (!stopped) {
                return new ChassisSpeeds();
            }
            m_phase = Phase.DRIVE_TO_POSITION;
        }

        double speed = Math.min(kHopperAlignMaxSpeedMetersPerSecond, kHopperAlignPositionP * distance);
        return new ChassisSpeeds(m_direction * speed * Math.cos(bearingError), 0.0, turnRate(bearingError));
    }

    private static double turnRate(double errorRadians) {
        return MathUtil.clamp(kHopperAlignHeadingP * errorRadians,
            -kHopperAlignMaxTurnRateRadPerSec, kHopperAlignMaxTurnRateRadPerSec);
    }

    public boolean isFinished() {
        return m_phase == Phase.FINISHED;
    }
}
