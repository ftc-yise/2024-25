package org.firstinspires.ftc.teamcode.Swerve;

/**
 * Simple 2D translation / vector class.
 * Unit-agnostic (meters/inches) — keep consistent across your project.
 */
public class Translation2d {
    private final double x;
    private final double y;

    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public Translation2d plus(Translation2d other) {
        return new Translation2d(this.x + other.x, this.y + other.y);
    }

    public Translation2d minus(Translation2d other) {
        return new Translation2d(this.x - other.x, this.y - other.y);
    }

    public Translation2d times(double scalar) {
        return new Translation2d(this.x * scalar, this.y * scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public Translation2d rotateBy(Rotation2d rot) {
        double cos = Math.cos(rot.getRadians());
        double sin = Math.sin(rot.getRadians());
        return new Translation2d(cos * x - sin * y, sin * x + cos * y);
    }

    @Override
    public String toString() {
        return String.format("Translation2d(%.6f, %.6f)", x, y);
    }
}
