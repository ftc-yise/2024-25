package org.firstinspires.ftc.teamcode.archived23_24SeaonCenterStage.demoBotDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Romeo strafe drive", group="Linear OpMode")
public class RomeoStrafeDriveCenterStage extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo Claw = null;
    private Servo Shoulder = null;

    private double slowSpeed = 0.4;
    private double fullSpeed = 1;
    private double currentSpeed = .35;
    private boolean canChangeSpeeds = true;
    private boolean RightTriggerPressed = false;
    private boolean LeftTriggerPressed = false;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        Shoulder = hardwareMap.get(Servo.class, "shoulder");
        Claw = hardwareMap.get(Servo.class, "claw");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward   = gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double strafe =  -gamepad1.left_stick_y;
            double turn     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = -forward + strafe + turn;
            double rightFrontPower = -forward - strafe - turn;
            double leftBackPower   = forward - strafe + turn;
            double rightBackPower  = forward + strafe - turn;

            /*if (!gamepad1.right_bumper) {
                RightTriggerPressed = false; // Reset the flag when the trigger is released
            }// Check the conditions for opening/closing the claw
            if (gamepad1.right_bumper && !RightTriggerPressed) {
                // If the right trigger is pressed, toggle the claw and reset the sensor flag
                Claw.setPosition(Claw.getPosition() == 0.7 ? 0.4 : 0.7);
                RightTriggerPressed = true; // Reset the flag when the trigger is released
            }*/

            //controlling shoulder when grabbing specimen of the wall position using a
            // toggle boolean and a ternary operator
            if (gamepad1.right_trigger < 0.15) {
                RightTriggerPressed = false; // Reset the flag when the trigger is released
            }// Check the conditions for opening/closing the claw
            if (gamepad1.right_trigger > 0.15 && !RightTriggerPressed) {
                // If the right trigger is pressed, toggle the claw and reset the sensor flag
                Claw.setPosition(Claw.getPosition() == 0.8 ? 0.2 : 0.8);
                RightTriggerPressed = true; // Reset the flag when the trigger is released
            }

            if (gamepad1.left_trigger < 0.15) {
                LeftTriggerPressed = false; // Reset the flag when the trigger is released
            }// Check the conditions for opening/closing the claw
            if (gamepad1.left_trigger > 0.15 && !LeftTriggerPressed) {
                // If the right trigger is pressed, toggle the claw and reset the sensor flag
                Shoulder.setPosition(Shoulder.getPosition() == 0.8 ? 0 : 0.8);
                LeftTriggerPressed = true; // Reset the flag when the trigger is released
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * currentSpeed);
            rightFrontDrive.setPower(rightFrontPower * currentSpeed);
            leftBackDrive.setPower(leftBackPower * currentSpeed);
            rightBackDrive.setPower(rightBackPower * currentSpeed);

            telemetry.addData("serve", Claw.getPosition());
            telemetry.addData("right trigger", RightTriggerPressed);
            telemetry.addData("shoulder", Claw.getPosition());
            telemetry.addData("left trigger", RightTriggerPressed);
            telemetry.update();
        }
    }}
