package org.firstinspires.ftc.teamcode.Swerve;

/**
 * Minimal rotation wrapper (radians internally). Utility helpers included.
 */
public class Rotation2d {
    private final double radians;

    public Rotation2d(double radians) {
        this.radians = normalizeRadians(radians);
    }

    public static Rotation2d fromRadians(double r) { return new Rotation2d(r); }
    public static Rotation2d fromDegrees(double d) { return new Rotation2d(Math.toRadians(d)); }

    public double getRadians() { return radians; }
    public double getDegrees() { return Math.toDegrees(radians); }

    public Rotation2d plus(Rotation2d other) {
        return new Rotation2d(this.radians + other.radians);
    }

    public Rotation2d minus(Rotation2d other) {
        return new Rotation2d(this.radians - other.radians);
    }

    public Rotation2d rotateBy(Rotation2d other) {
        return plus(other);
    }

    /**
     * Normalize angle into range [-pi, pi)
     */
    public static double normalizeRadians(double a) {
        double res = a % (2.0 * Math.PI);
        if (res >= Math.PI) res -= 2.0 * Math.PI;
        if (res < -Math.PI) res += 2.0 * Math.PI;
        return res;
    }

    /**
     * Smallest signed difference angle from a to b (b - a) in [-pi, pi).
     */
    public static double smallestAngleDiff(double a, double b) {
        return normalizeRadians(b - a);
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(%.6f rad / %.3f deg)", radians, getDegrees());
    }
}
