package org.firstinspires.ftc.teamcode.vision.detectRedPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core .*;
import java.util.ArrayList;


public class detectRedPipeline extends OpenCvPipeline {

        public Scalar lowerRGBA = new Scalar(29.0, 0.0, 16.0, 0.0);
        public Scalar upperRGBA = new Scalar(255.0, 70.0, 70.0, 255.0);
        private Mat rgbaBinaryMat = new Mat();

        private ArrayList<MatOfPoint> contours = new ArrayList<>();
        private Mat hierarchy = new Mat();

        private ArrayList<Rect> contoursRects = new ArrayList<>();

        private Rect biggestRect = null;

        public Scalar lineColor = new Scalar(255.0, 0.0, 0.0, 0.0);
        public int lineThickness = 1;

        private Mat rgbaBinaryMatRects = new Mat();

        public Scalar lineColor1 = new Scalar(255.0, 255.0, 255.0, 0.0);
        public int lineThickness1 = 0;

        private Mat rgbaBinaryMatContours = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Core.inRange(input, lowerRGBA, upperRGBA, rgbaBinaryMat);

            contours.clear();
            hierarchy.release();
            Imgproc.findContours(rgbaBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            contoursRects.clear();
            for (MatOfPoint points : contours) {
                contoursRects.add(Imgproc.boundingRect(points));
            }

            this.biggestRect = null;
            for (Rect rect : contoursRects) {
                if (rect != null) {
                    if ((biggestRect == null) || (rect.area() > biggestRect.area())) {
                        this.biggestRect = rect;
                    }
                }
            }

            rgbaBinaryMat.copyTo(rgbaBinaryMatRects);
            if (biggestRect != null) {
                Imgproc.rectangle(rgbaBinaryMatRects, biggestRect, lineColor, lineThickness);
            }

            rgbaBinaryMat.copyTo(rgbaBinaryMatContours);
            Imgproc.drawContours(rgbaBinaryMatContours, contours, -1, lineColor1, lineThickness1);

            return rgbaBinaryMatRects;
        }
    }