package org.firstinspires.ftc.teamcode.yise;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class OpenCVVision {
    //used to detect if the camera turns on or not
    boolean Error = false;

    private Servo outtake;

    OpenCvWebcam webcam;
    detectBluePipeline pipelineB;
    detectRedPipeline pipelineR;
    detectYellowPIpeline pipelineY;

    plainVisionPipeline PVP;


    // Used to set which color block to look for.
    public enum Color {
        RED,
        BLUE,
        YELLOW,
        TEST
    }

    private Color currentColor;
    public OpenCVVision(){

    }


    //Main Constructor
    public OpenCVVision(HardwareMap hardwareMap) {
        // Initialize Servo
        outtake = hardwareMap.get(Servo.class, "outtake");

        outtake.setPosition(-1);

        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Set the pipeline
        pipelineR = new detectRedPipeline();
        pipelineB = new detectBluePipeline();
        pipelineY = new detectYellowPIpeline();

        PVP = new plainVisionPipeline();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 25);

        // Set the pipeline
        webcam.setPipeline(pipelineY);
        currentColor = Color.YELLOW;

        // Open the camera device asynchronously
        webcam.setMillisecondsPermissionTimeout(2000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Error = true;
            }
        });
    }


    // Metrics for telemetry
    static double largestContourArea = 0;
    static double secondLargestContourArea = 0;
    static Point largestContourCentroid = new Point(0, 0);
    static Point topRightCorner = new Point(0, 0);
    static Point bottomRightCorner = new Point(0, 0);
    static double lineAngle = 0;
    static double perpendicularAngle1 = 0;
    static double perpendicularAngle2 = 0;


    // Define Yellow detecting pipeline as a non-static inner class
        public static class detectRedPipeline extends OpenCvPipeline {
            Mat blackBackground = new Mat();
            Mat contoursOnBlackBackground = new Mat();
            Mat lineTest = new Mat();
            List<MatOfPoint> contoursList = new ArrayList<>();

            // Enum for stages
            enum Stage {
                CONTOURS_ON_BLACK_BACKGROUND,
                RAW_IMAGE,
                LINE_TEST
            }

            private Stage stageToRenderToViewport = Stage.CONTOURS_ON_BLACK_BACKGROUND;
            private Stage[] stages = Stage.values();

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
                // Convert the image to HSV color space
                Mat hsv = new Mat();
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

                // Define yellow color range in HSV
                Scalar lowerYellow = new Scalar(20, 40, 40); // Adjust these values as needed
                Scalar upperYellow = new Scalar(120, 230, 255);

                // Threshold the HSV image to get only yellow colors
                Mat yellowMask = new Mat();
                Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

                // Find contours
                contoursList.clear();
                Imgproc.findContours(yellowMask, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                // Sort contours by area in descending order
                List<ContourArea> contourAreas = new ArrayList<>();
                for (MatOfPoint contour : contoursList) {
                    double area = Imgproc.contourArea(contour);
                    contourAreas.add(new ContourArea(contour, area));
                }
                contourAreas.sort((c1, c2) -> Double.compare(c2.area, c1.area)); // Descending order

                // Draw contours on the black background
                blackBackground.release();
                blackBackground = Mat.zeros(input.size(), input.type());
                contoursOnBlackBackground.release();
                blackBackground.copyTo(contoursOnBlackBackground);
                Imgproc.drawContours(contoursOnBlackBackground, contoursList, -1, new Scalar(0, 0, 255), 2);

                // Initialize variables for the largest and second-largest contours
                MatOfPoint largestContour = null;
                MatOfPoint secondLargestContour = null;
                largestContourArea = 0;
                secondLargestContourArea = 0;

                if (contourAreas.size() > 0) {
                    // Largest contour
                    largestContour = contourAreas.get(0).contour;
                    largestContourArea = contourAreas.get(0).area;

                    if (contourAreas.size() > 1) {
                        // Second-largest contour
                        secondLargestContour = contourAreas.get(1).contour;
                        secondLargestContourArea = contourAreas.get(1).area;
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
        //getter method
        public int getContourCount() {
            return contoursList.size();
        }
        }

    // Define Blue detecting pipeline as a non-static inner class
    public static class detectBluePipeline extends OpenCvPipeline {
        Mat blackBackground = new Mat();
        Mat contoursOnBlackBackground = new Mat();
        Mat lineTest = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        // Enum for stages
        enum Stage {
            CONTOURS_ON_BLACK_BACKGROUND,
            RAW_IMAGE,
            LINE_TEST
        }

        private Stage stageToRenderToViewport = Stage.CONTOURS_ON_BLACK_BACKGROUND;
        private Stage[] stages = Stage.values();

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
            // Convert the image to HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define the range for the color blue in HSV
            Scalar lowerBound = new Scalar(50, 80, 150); // Lower bound of the blue color range
            Scalar upperBound = new Scalar(140, 255, 255); // Upper bound of the blue color range

            // Threshold the HSV image to get only the blue color
            Mat blueMask = new Mat();
            Core.inRange(hsv, lowerBound, upperBound, blueMask);

            // Find contours
            contoursList.clear();
            Imgproc.findContours(blueMask, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Sort contours by area in descending order
            List<ContourArea> contourAreas = new ArrayList<>();
            for (MatOfPoint contour : contoursList) {
                double area = Imgproc.contourArea(contour);
                contourAreas.add(new ContourArea(contour, area));
            }
            contourAreas.sort((c1, c2) -> Double.compare(c2.area, c1.area)); // Descending order

            // Draw contours on the black background
            blackBackground.release();
            blackBackground = Mat.zeros(input.size(), input.type());
            contoursOnBlackBackground.release();
            blackBackground.copyTo(contoursOnBlackBackground);
            Imgproc.drawContours(contoursOnBlackBackground, contoursList, -1, new Scalar(0, 0, 255), 2);

            // Initialize variables for the largest and second-largest contours
            MatOfPoint largestContour = null;
            MatOfPoint secondLargestContour = null;
            largestContourArea = 0;
            secondLargestContourArea = 0;

            if (contourAreas.size() > 0) {
                // Largest contour
                largestContour = contourAreas.get(0).contour;
                largestContourArea = contourAreas.get(0).area;

                if (contourAreas.size() > 1) {
                    // Second-largest contour
                    secondLargestContour = contourAreas.get(1).contour;
                    secondLargestContourArea = contourAreas.get(1).area;
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
        //getter method
        public int getContourCount() {
            return contoursList.size();
        }
    }


    // Define Red detecting pipeline as a non-static inner class
    public static class detectYellowPIpeline extends OpenCvPipeline {
        Mat blackBackground = new Mat();
        Mat contoursOnBlackBackground = new Mat();
        Mat lineTest = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        // Enum for stages
        enum Stage {
            CONTOURS_ON_BLACK_BACKGROUND,
            RAW_IMAGE,
            LINE_TEST
        }

        private Stage stageToRenderToViewport = Stage.CONTOURS_ON_BLACK_BACKGROUND;
        private Stage[] stages = Stage.values();

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
            // Convert the image to YCrCb color space
            Mat yCrCb = new Mat();
            Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Extract the Cr and Y channels
            Mat crChannel = new Mat();
            Core.extractChannel(yCrCb, crChannel, 1); // Cr channel is index 1
            Mat yChannel = new Mat();
            Core.extractChannel(yCrCb, yChannel, 0); // Y channel is index 0

            // Threshold Cr channel to detect yellow
            Mat crThreshold = new Mat();
            Imgproc.threshold(crChannel, crThreshold, 175, 255, Imgproc.THRESH_BINARY); // Adjust the lower bound as needed

            // Threshold Y channel to ensure proper brightness
            Mat yThreshold = new Mat();
            Imgproc.threshold(yChannel, yThreshold, 100, 255, Imgproc.THRESH_BINARY); // Adjust the lower bound as needed

            // Combine the Cr and Y channel thresholds
            Mat combinedThreshold = new Mat();
            Core.bitwise_and(crThreshold, yThreshold, combinedThreshold);

            // Find contours
            contoursList.clear();
            Imgproc.findContours(combinedThreshold, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Sort contours by area in descending order
            List<ContourArea> contourAreas = new ArrayList<>();
            for (MatOfPoint contour : contoursList) {
                double area = Imgproc.contourArea(contour);
                contourAreas.add(new ContourArea(contour, area));
            }
            contourAreas.sort((c1, c2) -> Double.compare(c2.area, c1.area)); // Descending order

            // Draw contours on the black background
            blackBackground.release();
            blackBackground = Mat.zeros(input.size(), input.type());
            contoursOnBlackBackground.release();
            blackBackground.copyTo(contoursOnBlackBackground);
            Imgproc.drawContours(contoursOnBlackBackground, contoursList, -1, new Scalar(0, 0, 255), 2);

            // Initialize variables for the largest and second-largest contours
            MatOfPoint largestContour = null;
            MatOfPoint secondLargestContour = null;
            largestContourArea = 0;
            secondLargestContourArea = 0;

            if (contourAreas.size() > 0) {
                // Largest contour
                largestContour = contourAreas.get(0).contour;
                largestContourArea = contourAreas.get(0).area;

                if (contourAreas.size() > 1) {
                    // Second-largest contour
                    secondLargestContour = contourAreas.get(1).contour;
                    secondLargestContourArea = contourAreas.get(1).area;
                }

                // Compute the minimum-area rectangle for the second-largest contour, if it exists
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
        //getter method
        public int getContourCount() {
            return contoursList.size();
        }

    }


    // Inner class to store contour and its area
        private static class ContourArea {
            MatOfPoint contour;
            double area;

            ContourArea(MatOfPoint contour, double area) {
                this.contour = contour;
                this.area = area;
            }
        }

    class plainVisionPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }
    }

    //Set which color we should detect
    public void setCameraPipeline(Color colorDetection){
        switch (colorDetection) {
            case RED:
                webcam.setPipeline(pipelineR);
                currentColor = Color.RED;
                break;
            case BLUE:
                webcam.setPipeline(pipelineB);
                currentColor = Color.BLUE;
                break;
            case YELLOW:
                webcam.setPipeline(pipelineY);
                currentColor = Color.YELLOW;
                break;
            case TEST:
                webcam.setPipeline(PVP);
                currentColor = Color.TEST;
                break;
        }
    }

    // Getter methods for telemetry
    public double getLargestContourArea() {
        return largestContourArea;
    }

    public double getSecondLargestContourArea() {
        return secondLargestContourArea;
    }

    public Point getLargestContourCentroid() {
        return largestContourCentroid;
    }

    public Point getTopRightCorner() {
        return topRightCorner;
    }

    public Point getBottomRightCorner() {
        return bottomRightCorner;
    }

    public double getLineAngle() {
        return lineAngle;
    }

    public double getPerpendicularAngle1() {
        return perpendicularAngle1;
    }

    public double getPerpendicularAngle2() {
        return perpendicularAngle2;
    }

    public Color getColor(){
        return currentColor;
    }
}