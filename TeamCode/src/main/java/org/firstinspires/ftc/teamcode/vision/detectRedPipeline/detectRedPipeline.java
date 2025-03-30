package org.firstinspires.ftc.teamcode.vision.detectRedPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class detectRedPipeline extends OpenCvPipeline {

    private Mat blackBackground = new Mat();
    private Mat contoursOnBlackBackground = new Mat();
    private Mat lineTest = new Mat();
    private List<MatOfPoint> contoursList = new ArrayList<>();

    // Enum for stages
    enum Stage {
        CONTOURS_ON_BLACK_BACKGROUND,
        RAW_IMAGE,
        LINE_TEST
    }

    private Stage stageToRenderToViewport = Stage.CONTOURS_ON_BLACK_BACKGROUND;
    private Stage[] stages = Stage.values();

    // Red color range in HSV
    private Scalar lowerRED = new Scalar(0, 126, 30);  // Lower bound for red
    private Scalar upperRED = new Scalar(4.3, 255, 255);  // Upper bound for red

    // For line calculations
    private Point topRightCorner;
    private Point bottomRightCorner;
    private double lineAngle;
    private double perpendicularAngle1;
    private double perpendicularAngle2;
    private double largestContourArea;
    private double secondLargestContourArea;

    @Override
    public void onViewportTapped() {
        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert the image to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold the HSV image to get only red colors
        Mat redMask = new Mat();
        Core.inRange(hsv, lowerRED, upperRED, redMask);

        // Find contours
        contoursList.clear();
        Imgproc.findContours(redMask, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Sort contours by area in descending order
        contoursList.sort((c1, c2) -> Double.compare(Imgproc.contourArea(c2), Imgproc.contourArea(c1)));

        // Draw contours on the black background
        blackBackground.release();
        blackBackground = Mat.zeros(input.size(), input.type());
        contoursOnBlackBackground.release();
        blackBackground.copyTo(contoursOnBlackBackground);
        Imgproc.drawContours(contoursOnBlackBackground, contoursList, -1, new Scalar(0, 0, 255), 2);

        // Initialize variables for largest and second-largest contours
        MatOfPoint largestContour = null;
        MatOfPoint secondLargestContour = null;
        largestContourArea = 0;
        secondLargestContourArea = 0;

        if (contoursList.size() > 0) {
            // Largest contour
            largestContour = contoursList.get(0);
            largestContourArea = Imgproc.contourArea(largestContour);

            if (contoursList.size() > 1) {
                // Second largest contour
                secondLargestContour = contoursList.get(1);
                secondLargestContourArea = Imgproc.contourArea(secondLargestContour);
            }

            // Compute the minimum-area rectangle for the largest contour
            if (largestContour != null) {
                RotatedRect minAreaRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

                // Get the box points
                Point[] boxPoints = new Point[4];
                MatOfPoint2f boxPointsMat = new MatOfPoint2f();
                Imgproc.boxPoints(minAreaRect, boxPointsMat);
                boxPoints = boxPointsMat.toArray();

                // Draw the bounding box on the contours
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(contoursOnBlackBackground, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                // Calculate the angle of the line
                topRightCorner = boxPoints[1];
                bottomRightCorner = boxPoints[2];
                lineAngle = calculateAngle(boxPoints[0], boxPoints[2]);

                // Calculate perpendicular angles
                perpendicularAngle1 = (lineAngle + 90) % 360;
                perpendicularAngle2 = (lineAngle - 90) % 360;

                // Normalize angles to be within [0, 360) degrees
                if (perpendicularAngle1 < 0) perpendicularAngle1 += 360;
                if (perpendicularAngle2 < 0) perpendicularAngle2 += 360;
            }
        }

        // Choose the correct stage to render
        switch (stageToRenderToViewport) {
            case CONTOURS_ON_BLACK_BACKGROUND: {
                return contoursOnBlackBackground;
            }
            case RAW_IMAGE: {
                Mat rawWithContours = new Mat();
                input.copyTo(rawWithContours);
                Imgproc.drawContours(rawWithContours, contoursList, -1, new Scalar(0, 0, 255), 2);
                return rawWithContours;
            }
            case LINE_TEST: {
                return lineTest;
            }
            default: {
                return input;
            }
        }
    }

    // Method to calculate the angle between two points
    private double calculateAngle(Point p1, Point p2) {
        // Compute the difference in coordinates
        double deltaY = p2.y - p1.y;
        double deltaX = p2.x - p1.x;

        // Calculate the angle in radians and convert to degrees
        double angleRadians = Math.atan2(deltaY, deltaX);
        double angleDegrees = Math.toDegrees(angleRadians);

        // Normalize angle to be within 0 to 360 degrees
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        return angleDegrees;
    }

    // Getter for the contour count
    public int getContourCount() {
        return contoursList.size();
    }
}
