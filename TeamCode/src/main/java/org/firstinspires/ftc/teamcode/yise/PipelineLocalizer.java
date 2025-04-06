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

import org.firstinspires.ftc.teamcode.vision.detectRedPipeline.detectRedPipeline;

import java.util.ArrayList;
import java.util.List;

public class PipelineLocalizer {
    //used to detect if the camera turns on or not
    boolean Error = false;

    OpenCvWebcam webcam;
    detectRedPipeline pipelineR;

    // Used to set which color block to look for.
    public enum Color {
        RED,
        BLUE,
        YELLOW,
        TEST
    }

    private Color currentColor;

    //Main Constructor
    public PipelineLocalizer(HardwareMap hardwareMap) {

        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Set the pipeline
        pipelineR = new detectRedPipeline();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 25);

        // Set the pipeline
        webcam.setPipeline(pipelineR);
        currentColor = Color.BLUE;

        // Open the camera device asynchronously
        webcam.setMillisecondsPermissionTimeout(2000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Error = true;
            }
        });
    }

    //Set which color we should detect
    public void setCameraPipeline(Color colorDetection){
        switch (colorDetection) {
            case RED:
                webcam.setPipeline(pipelineR);
                currentColor = Color.RED;
                break;
            case BLUE:
                //applie blue pipeline
                break;
            case YELLOW:
                //applie yellow pipeline
                break;
            case TEST:
                //applie a test pipeline
                break;
        }
    }

    public Color getColor(){
        return currentColor;
    }
}