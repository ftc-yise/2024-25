package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LiftClass;
import org.firstinspires.ftc.teamcode.yise.ledLights;
// import org.firstinspires.ftc.teamcode.yise.Parameters;
// import org.firstinspires.ftc.teamcode.yise.poseStorage;

@Autonomous(name = "yellowAutoBlueBucket", group = "Linear Opmode")

public class autoBlueBucket extends LinearOpMode {
    public float endLocation_X = 0;
    public float endLocation_Y = 16;
    public float endHeading_Z = 90;

    @Override
    public void runOpMode() throws InterruptedException {

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
        TrajectorySequence place_Block_1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(55, 52, Math.toRadians(225)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence pick_Up_Block_2 = drive.trajectorySequenceBuilder(place_Block_1.end())
                .lineToLinearHeading(new Pose2d(48, 40, Math.toRadians(270)))
                .forward(8)
                .waitSeconds(0.51)
                .build();

        TrajectorySequence place_Block_2 = drive.trajectorySequenceBuilder(pick_Up_Block_2.end())
                .lineToLinearHeading(new Pose2d(55, 52, Math.toRadians(225)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence pick_Up_Block_3 = drive.trajectorySequenceBuilder(place_Block_2.end())
                .lineToLinearHeading(new Pose2d(59, 40, Math.toRadians(270)))
                .forward(8)
                .waitSeconds(0.5)
                .build();


        TrajectorySequence place_Block_3 = drive.trajectorySequenceBuilder(pick_Up_Block_3.end())
                .lineToLinearHeading(new Pose2d(55, 52, Math.toRadians(225)))
                .waitSeconds(0.5)
                .build();


        // run my trajectories in order

        // telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        // telemetry.addData("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();


        drive.followTrajectorySequence(place_Block_1);

        arm.setElbowPosition(0.3);
        arm.setShoulderPosition(1);

        arm.setLiftPower(0.5);
        while (arm.getLiftPositionL() <400) {
            sleep(50);
        }
        arm.setLiftPower(0.08);
        arm.setPulleyPosition(LiftClass.pulleyPosition.BASKET);

        sleep(2300);

        arm.setElbowPosition(0.6);
        arm.setShoulderPosition(1);
        sleep(400);

        arm.setClawPosition(1);

        sleep(150);

        arm.setElbowPosition(0.5);
        arm.setShoulderPosition(0.9);
        sleep(250);

        arm.setElbowPosition(0);
        arm.setShoulderPosition(0.4);
        sleep(250);

        arm.setPulleyPosition(LiftClass.pulleyPosition.HOME);

        sleep(1500);
        arm.setLiftPosition(LiftClass.liftPosition.HOME);
        sleep(250);

        arm.setPulleyPower(0);

        drive.followTrajectorySequence(pick_Up_Block_2);

        arm.setShoulderPosition(0.65);
        arm.setElbowPosition(0.12);
        sleep(500);
        arm.CloseClaw();



        drive.followTrajectorySequence(place_Block_2);

        arm.setElbowPosition(0.3);
        arm.setShoulderPosition(1);

        arm.setLiftPower(0.5);
        while (arm.getLiftPositionL() <400) {
            sleep(50);
        }
        arm.setLiftPower(0.08);
        arm.setPulleyPosition(LiftClass.pulleyPosition.BASKET);

        sleep(2300);

        arm.setElbowPosition(0.6);
        arm.setShoulderPosition(1);
        sleep(400);

        arm.setClawPosition(1);

        sleep(150);

        arm.setElbowPosition(0.5);
        arm.setShoulderPosition(0.9);
        sleep(250);

        arm.setElbowPosition(0);
        arm.setShoulderPosition(0.4);
        sleep(250);

        arm.setPulleyPosition(LiftClass.pulleyPosition.HOME);

        sleep(1500);


        arm.setLiftPosition(LiftClass.liftPosition.HOME);

        sleep(250);
        arm.setPulleyPower(0);

        drive.followTrajectorySequence(pick_Up_Block_3);
        arm.setShoulderPosition(0.6);
        arm.setElbowPosition(0.12);
        sleep(250);
        arm.CloseClaw();

        drive.followTrajectorySequence(place_Block_3);
        arm.setElbowPosition(0.3);
        arm.setShoulderPosition(1);

        arm.setLiftPower(0.5);
        while (arm.getLiftPositionL() <400) {
            sleep(50);
        }
        arm.setLiftPower(0.08);
        arm.setPulleyPosition(LiftClass.pulleyPosition.BASKET);
        sleep(2300);

        arm.setElbowPosition(0.6);
        arm.setShoulderPosition(1);
        sleep(400);

        arm.setClawPosition(1);

        sleep(150);

        arm.setElbowPosition(0.5);
        arm.setShoulderPosition(0.9);
        sleep(250);

        arm.setElbowPosition(0);
        arm.setShoulderPosition(0.4);
        sleep(250);

        arm.setPulleyPosition(LiftClass.pulleyPosition.HOME);

        sleep(1500);
        arm.setLiftPosition(LiftClass.liftPosition.HOME);
        sleep(250);

        arm.setPulleyPower(0);

        sleep(250);

        //drive.followTrajectorySequence(park_At_Submersible_And_Hang);
        // telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        // telemetry.addData ("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        //drive.followTrajectorySequence(seq_2);
}}
