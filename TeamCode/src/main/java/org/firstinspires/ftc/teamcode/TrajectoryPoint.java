package org.firstinspires.ftc.teamcode;

public class TrajectoryPoint {
    public final double t;   // seconds
    public final double x;   // inches (generator output)
    public final double y;   // inches
    public final double th;  // radians

    public TrajectoryPoint(double t, double x, double y, double th) {
        this.t = t;
        this.x = x;
        this.y = y;
        this.th = th;
    }

    /** interpolate between a->b at fraction u in [0,1], angles use shortest wrap */
    public static TrajectoryPoint interp(TrajectoryPoint a, TrajectoryPoint b, double u) {
        double t = a.t + (b.t - a.t) * u;
        double x = a.x + (b.x - a.x) * u;
        double y = a.y + (b.y - a.y) * u;
        // shortest angle interp
        double d = shortestAngleDiff(a.th, b.th);
        double th = a.th + d * u;
        return new TrajectoryPoint(t, x, y, th);
    }

    /** returns shortest signed difference b - a in [-pi,pi] */
    public static double shortestAngleDiff(double a, double b) {
        double diff = (b - a + Math.PI) % (2.0*Math.PI);
        if (diff < 0) diff += 2.0*Math.PI;
        diff -= Math.PI;
        return diff;
    }
}
