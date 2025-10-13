package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d poseEstimate = poseHistory.get(i);
            xPoints[i] = poseEstimate.getX();
            yPoints[i] = poseEstimate.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d poseEstimate = path.get(displacement);
            xPoints[i] = poseEstimate.getX();
            yPoints[i] = poseEstimate.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d poseEstimate) {
        canvas.strokeCircle(poseEstimate.getX(), poseEstimate.getY(), ROBOT_RADIUS);
        Vector2d v = poseEstimate.headingVec().times(ROBOT_RADIUS);
        double x1 = poseEstimate.getX() + v.getX() / 2, y1 = poseEstimate.getY() + v.getY() / 2;
        double x2 = poseEstimate.getX() + v.getX(), y2 = poseEstimate.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
