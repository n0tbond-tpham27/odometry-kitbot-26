package frc.robot.vision;

import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Steer back onto the hopper target, creating forward clearance before a large sideways correction. */
public final class HopperAlignmentController {
    private enum Phase { CLEARANCE, APPROACH, FINAL_HEADING, FINISHED }

    private Pose2d m_target;
    private final double m_positionToleranceMeters;
    private final double m_maxSpeed;
    private Phase m_phase = Phase.APPROACH;

    public HopperAlignmentController(Pose2d target) {
        this(target, kHopperAlignToleranceMeters);
    }

    public HopperAlignmentController(Pose2d target, double positionToleranceMeters) {
        this(target, positionToleranceMeters, kHopperAlignMaxSpeedMetersPerSecond);
    }

    public HopperAlignmentController(Pose2d target, double positionToleranceMeters, double maxSpeed) {
        m_target = target;
        m_positionToleranceMeters = positionToleranceMeters;
        m_maxSpeed = maxSpeed;
    }

    public void reset(Pose2d current) {
        m_phase = needsClearance(current.relativeTo(m_target)) ? Phase.CLEARANCE : Phase.APPROACH;
    }

    /** Refine the lateral goal from the observed image without restarting a clearance maneuver. */
    public void updateTarget(Pose2d target, Pose2d current) {
        if (target.getTranslation().getDistance(m_target.getTranslation()) > 0.004) {
            m_target = target;
            if (m_phase == Phase.FINISHED) {
                reset(current);
            }
        }
    }

    private boolean needsClearance(Pose2d relative) {
        // Move away first instead of making a 90-degree turn beside the hopper and losing tags.
        return relative.getX() < -m_positionToleranceMeters
            || (Math.abs(relative.getY()) > kHopperAlignLateralToleranceMeters
                && relative.getX() < Math.max(0.20, 2.0 * Math.abs(relative.getY())));
    }

    /** Recalculate from the latest vision-corrected pose; no sideways drive is assumed. */
    public ChassisSpeeds calculate(Pose2d current, ChassisSpeeds measuredSpeeds) {
        Pose2d relative = current.relativeTo(m_target);
        double distance = relative.getTranslation().getNorm();
        double finalHeadingError = MathUtil.angleModulus(
            m_target.getRotation().minus(current.getRotation()).getRadians());
        boolean stopped = Math.abs(measuredSpeeds.vxMetersPerSecond)
                < kDriveStraightSpeedToleranceMetersPerSecond
            && Math.abs(measuredSpeeds.omegaRadiansPerSecond) < kHopperAlignTurnSpeedToleranceRadPerSec;

        if (m_phase == Phase.FINISHED) {
            return new ChassisSpeeds();
        }
        if (m_phase == Phase.APPROACH && Math.abs(relative.getY()) > kHopperAlignLateralToleranceMeters
                && (relative.getX() <= m_positionToleranceMeters + kHopperApproachStopMarginMeters
                    || Math.abs(Math.atan2(relative.getY(), relative.getX())) > kHopperApproachMaxHeadingRadians)) {
            m_phase = Phase.CLEARANCE;
        }
        if (distance <= m_positionToleranceMeters && Math.abs(relative.getY()) <= kHopperAlignLateralToleranceMeters) {
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

        if (m_phase == Phase.CLEARANCE) {
            double clearance = Math.max(kHopperAlignClearanceMeters, 2.5 * Math.abs(relative.getY()));
            if (relative.getX() >= clearance - m_positionToleranceMeters) {
                m_phase = Phase.APPROACH;
            } else {
                double speed = Math.min(kHopperAlignClearanceSpeedMetersPerSecond,
                    kHopperAlignPositionP * (clearance - relative.getX()));
                if (relative.getX() < 0.25) {
                    // First pull straight away along the current heading. Turning beside the
                    // hub can swing an overhanging hopper into it even at low linear speed.
                    return new ChassisSpeeds(Math.abs(finalHeadingError) < Math.toRadians(75.0)
                        ? Math.min(speed, kHopperApproachMaxSpeedMetersPerSecond) : 0.0, 0.0, 0.0);
                }
                if (Math.abs(finalHeadingError) > Math.toRadians(25.0)) {
                    speed = 0.0;
                }
                return new ChassisSpeeds(speed, 0.0, turnRate(finalHeadingError));
            }
        }

        double bearing = Math.atan2(relative.getY(), Math.max(relative.getX(), 0.01));
        double alpha = MathUtil.angleModulus(bearing - relative.getRotation().getRadians());
        double desiredHeading = MathUtil.clamp(
            (1.0 + kHopperAlignFinalBearingP / kHopperAlignBearingP) * bearing,
            -kHopperApproachMaxHeadingRadians, kHopperApproachMaxHeadingRadians);
        double turn = kHopperAlignBearingP * MathUtil.angleModulus(
            desiredHeading - relative.getRotation().getRadians());
        double speedLimit = m_maxSpeed;
        if (relative.getX() < 0.8 || Math.abs(relative.getY()) > kHopperAlignToleranceMeters) {
            speedLimit = Math.min(speedLimit, kHopperApproachMaxSpeedMetersPerSecond);
        }
        speedLimit = Math.min(speedLimit, Math.sqrt(2.0 * kHopperApproachBrakingAccelMetersPerSecondSq
            * Math.max(0.0, relative.getX() - m_positionToleranceMeters / 2.0)));
        double speed = Math.min(speedLimit, kHopperAlignPositionP * distance) * Math.max(0.0, Math.cos(alpha));
        if (Math.abs(alpha) > Math.toRadians(25.0)) {
            speed = 0.0;
        }
        return new ChassisSpeeds(-speed, 0.0,
            MathUtil.clamp(turn, -kHopperAlignMaxTurnRateRadPerSec, kHopperAlignMaxTurnRateRadPerSec));
    }

    private static double turnRate(double errorRadians) {
        return MathUtil.clamp(kHopperAlignHeadingP * errorRadians,
            -kHopperAlignMaxTurnRateRadPerSec, kHopperAlignMaxTurnRateRadPerSec);
    }

    public boolean isFinished() {
        return m_phase == Phase.FINISHED;
    }

    public boolean isCreatingClearance() {
        return m_phase == Phase.CLEARANCE;
    }
}
