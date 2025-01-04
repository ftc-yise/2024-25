package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LiftClass;
import org.firstinspires.ftc.teamcode.yise.ledLights;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LiftClass;
import org.firstinspires.ftc.teamcode.yise.ledLights;
// import org.firstinspires.ftc.teamcode.yise.Parameters;
// import org.firstinspires.ftc.teamcode.yise.poseStorage;

@Autonomous(name = "autoRedPark", group = "Linear Opmode")

public class autoRedPark extends LinearOpMode {
    public float endLocation_X = 0;
    public float endLocation_Y = 16;
    public float endHeading_Z = 90;

    @Override
    public void runOpMode() {

        // ------------------------------------------------------------------------------------
        // Initialize Class Instances and Variables
        // ------------------------------------------------------------------------------------
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LiftClass arm = new LiftClass(hardwareMap);
        ledLights leds = new ledLights(hardwareMap);
        // poseStorage.currentPose = drive.getPoseEstimate();

        waitForStart();
        if(isStopRequested()) return;

        leds.setLed(ledLights.ledStates.INIT);

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(-23, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence red_Side_Park = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42, 12, Math.toRadians(0)))
                .forward(19)
                .build();

        // run my trajectories in order

        // telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        // telemetry.addData("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        // drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(red_Side_Park);
        // telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        // telemetry.addData ("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        //drive.followTrajectorySequence(seq_2);


    }
}

