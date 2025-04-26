package org.firstinspires.ftc.teamcode.vision.detectRedPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import java.util.ArrayList;
import java.util.Collections;

public class detectRedPipeline extends OpenCvPipeline {
    public Scalar lowerRGBA = new Scalar(79.0, 0.0, 0.0, 0.0);
    public Scalar upperRGBA = new Scalar(255.0, 46.8, 57.0, 255.0);
    private Mat rgbaBinaryMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private MatOfPoint2f contours2f = new MatOfPoint2f();

    private ArrayList<RotatedRect> contoursRotRects = new ArrayList<>();
    private RotatedRect largestRotatedRect = null;

    public Scalar lineColor = new Scalar(0.0, 0.0, 255.0, 0.0);
    public int lineThickness = 5;

    private Mat inputRotRect = new Mat();

    private Mat rgbaBinaryMatRotRects = new Mat();

    public Scalar lineColor1 = new Scalar(255.0, 255.0, 255.0, 0.0);
    public int lineThickness1 = 0;

    private Mat rgbaBinaryMatContours = new Mat();

    // Flag to indicate if any valid detection was found
    private boolean redObjectDetected = false;

    @Override
    public Mat processFrame(Mat input) {
        Core.inRange(input, lowerRGBA, upperRGBA, rgbaBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(rgbaBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursRotRects.clear();
        largestRotatedRect = null;
        double maxArea = 0;

        for(MatOfPoint points : contours) {
            contours2f.release();
            points.convertTo(contours2f, CvType.CV_32F);
            RotatedRect rect = Imgproc.minAreaRect(contours2f);
            contoursRotRects.add(rect);

            // Keep track of the largest rotated rectangle
            if (rect.size.area() > maxArea) {
                maxArea = rect.size.area();
                largestRotatedRect = rect;
            }
        }

        // Update detection flag
        redObjectDetected = largestRotatedRect != null;

        input.copyTo(inputRotRect);

        // Draw all rotated rectangles
        for(RotatedRect rect : contoursRotRects) {
            if(rect != null) {
                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint matOfPoint = new MatOfPoint(rectPoints);
                Imgproc.polylines(inputRotRect, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);
            }
        }

        // Draw the center point of the largest rectangle if it exists
        if (largestRotatedRect != null) {
            Imgproc.circle(inputRotRect, largestRotatedRect.center, 5, new Scalar(255, 0, 0), -1);
        }

        rgbaBinaryMat.copyTo(rgbaBinaryMatContours);
        Imgproc.drawContours(rgbaBinaryMatContours, contours, -1, lineColor1, lineThickness1);

        return inputRotRect;
    }

    // Method to check if red object is detected
    public boolean isRedObjectDetected() {
        return redObjectDetected;
    }

    // Get the number of detected contours
    public int getNumberOfDetectedObjects() {
        return contoursRotRects.size();
    }

    // Get all detected rotated rectangles
    public ArrayList<RotatedRect> getAllRotatedRects() {
        return contoursRotRects;
    }

    // Get the largest rotated rectangle
    public RotatedRect getLargestRotatedRect() {
        return largestRotatedRect;
    }

    // Get the width of the largest rotated rectangle
    public double getLargestRectWidth() {
        if (largestRotatedRect != null) {
            return largestRotatedRect.size.width;
        }
        return 0;
    }

    // Get the height of the largest rotated rectangle
    public double getLargestRectHeight() {
        if (largestRotatedRect != null) {
            return largestRotatedRect.size.height;
        }
        return 0;
    }

    // Get the angle of the largest rotated rectangle
    public double getLargestRectAngle() {
        if (largestRotatedRect != null) {
            return largestRotatedRect.angle;
        }
        return 0;
    }

    // Get the center point X coordinate of the largest rotated rectangle
    public double getLargestRectCenterX() {
        if (largestRotatedRect != null) {
            return largestRotatedRect.center.x;
        }
        return 0;
    }

    // Get the center point Y coordinate of the largest rotated rectangle
    public double getLargestRectCenterY() {
        if (largestRotatedRect != null) {
            return largestRotatedRect.center.y;
        }
        return 0;
    }
}