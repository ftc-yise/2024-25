package org.firstinspires.ftc.teamcode.Swerve;

/**
 * Minimal swerve kinematics implementation.
 * Construct with module locations as Translation2d (robot body frame), for example:
 *   new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight)
 *
 * toSwerveModuleStates computes wheel velocities in robot body frame:
 *   v_wheel = [vx - omega * ry, vy + omega * rx]
 */
public class SwerveDriveKinematics {
    private final Translation2d[] moduleLocations;

    public SwerveDriveKinematics(Translation2d... moduleLocations) {
        this.moduleLocations = moduleLocations.clone();
    }

    /**
     * Convert chassis speeds to module states (speed + angle).
     * Order of returned array matches constructor order.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = new SwerveModuleState[moduleLocations.length];

        for (int i = 0; i < moduleLocations.length; i++) {
            double rx = moduleLocations[i].getX();
            double ry = moduleLocations[i].getY();

            // wheel velocity in robot frame
            double vx = chassisSpeeds.vxMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond * ry;
            double vy = chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond * rx;

            double speed = Math.hypot(vx, vy);
            double angle = Math.atan2(vy, vx);
            states[i] = new SwerveModuleState(speed, Rotation2d.fromRadians(angle));
        }

        return states;
    }

    /**
     * Scale the wheel speeds uniformly if any exceeds maxSpeed.
     */
    public static void normalizeWheelSpeeds(SwerveModuleState[] states, double maxSpeed) {
        if (maxSpeed <= 0) return;
        double maxFound = 0.0;
        for (SwerveModuleState s : states) {
            maxFound = Math.max(maxFound, Math.abs(s.speedMetersPerSecond));
        }
        if (maxFound > maxSpeed) {
            double scale = maxSpeed / maxFound;
            for (SwerveModuleState s : states) {
                s.speedMetersPerSecond *= scale;
            }
        }
    }
}
