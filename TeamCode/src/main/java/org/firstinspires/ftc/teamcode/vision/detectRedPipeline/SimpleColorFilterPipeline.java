package org.firstinspires.ftc.teamcode.vision.detectRedPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SimpleColorFilterPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Mat output = new Mat();

        // Convert the input frame to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Define range for detecting red color
        Scalar lowerRed = new Scalar(0, 126, 30);  // Lower bound for red
        Scalar upperRed = new Scalar(4.3, 255, 255);

        // Threshold the image for red
        Core.inRange(hsv, lowerRed, upperRed, output);

        return output;
    }
}
