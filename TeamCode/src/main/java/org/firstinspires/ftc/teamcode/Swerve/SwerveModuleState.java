package org.firstinspires.ftc.teamcode.Swerve;

/**
 * Swerve module state: wheel speed and wheel angle.
 * Includes a small optimize() helper that minimizes wheel rotation by optionally reversing wheel direction.
 */
public class SwerveModuleState {
    public double speedMetersPerSecond;
    public Rotation2d angle;

    public SwerveModuleState(double speed, Rotation2d angle) {
        this.speedMetersPerSecond = speed;
        this.angle = angle;
    }

    /**
     * Optimize target state to be as close as possible to currentAngle.
     * If flipping the wheel by 180° (and inverting speed) results in smaller rotation, do that.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetRad = Rotation2d.normalizeRadians(desiredState.angle.getRadians());
        double currentRad = Rotation2d.normalizeRadians(currentAngle.getRadians());
        double diff = Rotation2d.smallestAngleDiff(currentRad, targetRad);

        // If rotating more than 90 degrees, it's shorter to flip wheel and invert speed
        if (Math.abs(diff) > Math.PI / 2.0) {
            double newSpeed = -desiredState.speedMetersPerSecond;
            double newAngleRad = Rotation2d.normalizeRadians(targetRad + Math.PI);
            return new SwerveModuleState(newSpeed, Rotation2d.fromRadians(newAngleRad));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromRadians(targetRad));
        }
    }

    @Override
    public String toString() {
        return String.format("SwerveModuleState(speed=%.4f, angle=%.3f°)", speedMetersPerSecond, angle.getDegrees());
    }
}
