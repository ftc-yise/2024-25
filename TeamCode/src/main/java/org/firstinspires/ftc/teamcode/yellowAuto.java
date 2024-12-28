package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LiftClass;
import org.firstinspires.ftc.teamcode.yise.ledLights;
// import org.firstinspires.ftc.teamcode.yise.Parameters;
// import org.firstinspires.ftc.teamcode.yise.poseStorage;

@Autonomous(name = "yellowAuto", group = "Linear Opmode")

@Disabled
public class yellowAuto extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(-38, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        int coneNumber = 3;
        coneNumber = tensor.readCone();

        if (coneNumber == 3){
            endLocation_X = -66;
            endLocation_Y = 14;
            endHeading_Z = 0;
        } else if (coneNumber == 2){
            endLocation_X = -40;
            endLocation_Y = 16;
            endHeading_Z = 0;
        } else if (coneNumber == 1){
            endLocation_X = -15;
            endLocation_Y = 16;
            endHeading_Z = 0;
        }

        // example trajectory sequences
        // based on testing, there's no problem doing forward and strafeRight in a row


        // seq_1:
        // Drive forward 20 inches (after first 10 inches, start raising arm to top cone height)
        // Then strafe right 20 inches and turn right 90 degrees
        //
        // This is an example of a "global" displacement marker, it actually doesn't matter where
        // it is placed in the order of the sequence. It will run after the bot has moved
        // a total of 10 inches.
        //
        // If you change the marker from 10 to 30, it will run in the middle of the stafeRight()
        // Note: 1 marker, when written in this syntax, can include multiple actions
        //Seq 1 = Start Path
        TrajectorySequence seq_1 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(-13)
                .splineToConstantHeading(new Vector2d(-50, 14), Math.toRadians(90))
                .addDisplacementMarker(20, () -> {
                    arm.getTopCone();
                })
                .addDisplacementMarker(20, () -> {
                    arm.openGrabber();
                })
                .turn(Math.toRadians(90))
                .forward(10)
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop(mecanumDrive.centerModes.CONE);
                })


                .addTemporalMarker(() -> {
                    arm.closeGrabber();
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    arm.setPoleHeight(liftArm.Heights.HIGH);
                })
                .build();


        TrajectorySequence scorecone_2 = drive.trajectorySequenceBuilder(seq_1.end())
                .lineToLinearHeading(new Pose2d(-10, 14, 45))
                .forward(2)
                .addTemporalMarker(() -> {
                    arm.openGrabber();
                })
                .waitSeconds(.6)
                .back(4)
                .build();

        TrajectorySequence seq_3 = drive.trajectorySequenceBuilder(scorecone_2.end())
                .turn(Math.toRadians(130))
                .lineToLinearHeading(new Pose2d(-52, 12, 0))
                .addDisplacementMarker(20, () -> {
                    arm.getTopCone();
                    arm.downOneCone();
                })
                .addTemporalMarker(() -> {
                    yiseDrive.autoCenterLoop(mecanumDrive.centerModes.CONE);
                })
                .waitSeconds(.2)
                .addTemporalMarker(() ->{
                    arm.closeGrabber();
                })
                .waitSeconds(.2)
                .addTemporalMarker(() ->{
                    arm.setPoleHeight(liftArm.Heights.HIGH);
                })
                .build();

        TrajectorySequence endposition_4 = drive.trajectorySequenceBuilder(scorecone_2.end())
                .lineToLinearHeading(new Pose2d( endLocation_X, endLocation_Y,  Math.toRadians(endHeading_Z)))
                .addTemporalMarker(() ->{
                    arm.closeGrabber();
                })
                .addTemporalMarker(() ->{
                    arm.returnToBottom();
                })
                .turn(Math.toRadians(90))
                .build();


        // run my trajectories in order

        telemetry.addData("cone#", coneNumber);
        telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        telemetry.addData("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();

        // drive to cone stack with arm at cone 5 height
        drive.followTrajectorySequence(seq_1);
        drive.followTrajectorySequence(scorecone_2);
        telemetry.addData("Distance S Left", yiseDrive.distanceSensorLeft);
        telemetry.addData ("Distance S Right", yiseDrive.distanceSensorRight);
        telemetry.update();
        drive.followTrajectorySequence(endposition_4);

        //drive.followTrajectorySequence(seq_2);


    }
}

