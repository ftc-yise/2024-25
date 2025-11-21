package org.firstinspires.ftc.teamcode.Swerve;

/**
 * Chassis velocity set: vx (forward), vy (left), omega (counterclockwise).
 * Unit-agnostic — keep consistent with wheel and kinematics units.
 */
public class ChassisSpeeds {
    public final double vxMetersPerSecond;
    public final double vyMetersPerSecond;
    public final double omegaRadiansPerSecond;

    public ChassisSpeeds(double vx, double vy, double omega) {
        this.vxMetersPerSecond = vx;
        this.vyMetersPerSecond = vy;
        this.omegaRadiansPerSecond = omega;
    }

    /**
     * Convert field-relative speeds to robot-relative given heading.
     * Heading is Rotation2d: 0 means robot forward aligned with field-forward.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(double vxField, double vyField, double omega, Rotation2d robotHeading) {
        // rotate field velocities into robot body frame: v_body = R(-heading) * v_field
        double cos = Math.cos(-robotHeading.getRadians());
        double sin = Math.sin(-robotHeading.getRadians());
        double vx = cos * vxField - sin * vyField;
        double vy = sin * vxField + cos * vyField;
        return new ChassisSpeeds(vx, vy, omega);
    }

    @Override
    public String toString() {
        return String.format("ChassisSpeeds(vx=%.4f, vy=%.4f, ω=%.4f)", vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
}
