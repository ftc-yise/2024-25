package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Ball Bot", group="Ball Bot")

public class BallBot extends LinearOpMode {
    SampleMecanumDrive drive;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor intake = null;

    private double slowSpeed = 0.4;
    private double fullSpeed = 1;
    private double currentSpeed = 1;
    private boolean canChangeSpeeds = true;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        double forward = 0;
        double strafe = 0;
        double turn = 0;



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            /*// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            forward   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            strafe =  gamepad1.left_stick_x;
            turn     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = forward + strafe + turn;
            double rightFrontPower = forward - strafe - turn;
            double leftBackPower   = forward - strafe + turn;
            double rightBackPower  = forward + strafe - turn;

            if (gamepad1.dpad_up){
                leftFrontDrive.setPower(1);
            } else if (gamepad1.dpad_down) {
                rightBackDrive.setPower(1);
            } else if (gamepad1.dpad_left) {
                leftBackDrive.setPower(1);
            } else if (gamepad1.dpad_right) {
                rightFrontDrive.setPower(1);
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * currentSpeed);
            rightFrontDrive.setPower(rightFrontPower * currentSpeed);
            leftBackDrive.setPower(leftBackPower * currentSpeed);
            rightBackDrive.setPower(rightBackPower * currentSpeed);*/

            if (gamepad1.y && canChangeSpeeds) {
                canChangeSpeeds = false;
                if (currentSpeed == fullSpeed) {
                    currentSpeed = slowSpeed;
                } else {
                    currentSpeed = fullSpeed;
                }
            } else if (!gamepad1.y) {
                canChangeSpeeds = true;
            }

            if (gamepad1.right_trigger > 0.75 || gamepad1.left_trigger > 0.75){
                intake.setPower(1);
            }else {
                intake.setPower(0);
            }

            //Set drive power based on gamepad inputs multiplied by the speed variable
            if (!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                strafe   = gamepad1.left_stick_y * currentSpeed;  // Note: pushing stick forward gives negative value
                forward =  gamepad1.left_stick_x * currentSpeed;
                turn     =  -gamepad1.right_stick_x * currentSpeed;
                drive.setWeightedDrivePower(new Pose2d(strafe, forward, turn));
            } else if (gamepad1.dpad_up) {
                drive.setWeightedDrivePower(new Pose2d(-.251, 0, 0));
            } else if (gamepad1.dpad_down) {
                drive.setWeightedDrivePower(new Pose2d(.251, 0, 0));
            } else if (gamepad1.dpad_left) {
                drive.setWeightedDrivePower(new Pose2d(0, -.251, 0));
            } else if (gamepad1.dpad_right) {
                drive.setWeightedDrivePower(new Pose2d(0, .251, 0));
            }

            drive.updateOTOS();

            Pose2d poseEstimate = drive.LDrive.getOTOSPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }
    }}
