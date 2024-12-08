package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import org.firstinspires.ftc.teamcode.yise.PoseStorage;

import org.firstinspires.ftc.teamcode.yise.OpenCVVision;

@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    double targetAngle = 0;
    double targetHeight = 0;

    boolean trigger = false;

    OpenCVVision vision = new OpenCVVision();


    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Pose2d targetAPose = new Pose2d(0, -31, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        // Ensure that the contents are copied over from https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/SampleMecanumDriveCancelable.java
        // and https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TrajectorySequenceRunnerCancelable.java
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        OpenCVVision vision = new OpenCVVision(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        PoseStorage.currentPose = new Pose2d(-32, -64, Math.toRadians(90));
        drive.setPoseEstimate(PoseStorage.currentPose);

        vision.setCameraPipeline(OpenCVVision.Color.RED);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("Status", "Initialized");

            telemetry.addLine();
            telemetry.addLine();

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode

            if (gamepad1.right_trigger < 0.75){
                trigger = false;
            }

            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * .75,
                                    -gamepad1.left_stick_x * .75,
                                    -gamepad1.right_stick_x * .75
                            )
                    );

                    if (gamepad1.y) {
                        // If the A button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(targetAPose)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}