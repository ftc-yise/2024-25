package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "autoBluePark", group = "Linear Opmode")

public class autoBluePark extends LinearOpMode {
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
        if (isStopRequested()) return;

        leds.setLed(ledLights.ledStates.INIT);

        // ------------------------------------------------------------------------------------
        // Define Trajectories and Arm/Grabber Actions
        // ------------------------------------------------------------------------------------

        // Start by defining our start position
        Pose2d startPose = new Pose2d(23, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence blue_Side_Park = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(42, -12, Math.toRadians(180)))
                .forward(20)
                .build();

        // run my trajectories in order

        // telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        // telemetry.addData("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        // drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(blue_Side_Park);
        // telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        // telemetry.addData ("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        //drive.followTrajectorySequence(seq_2);


    }}