package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.yise.ledLights;

@TeleOp(name="jack Motor Testing", group="Linear Opmode")
public class jackMotorTesting extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    public Servo servo1;

    private DcMotor left = null;
    private DcMotor right = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            telemetry.addData("Target position", "left position: " + left.getTargetPosition());
            telemetry.addData("Target position", "right position: " + right.getTargetPosition());
            telemetry.addData("Servo position", "servo1 position: " + servo1.getPosition());

            if (gamepad1.dpad_down) {
                left.setTargetPosition(100);
                right.setTargetPosition(100);
                left.setPower(0.3);
                right.setPower(0.3);
            } else if (gamepad1.dpad_up) {
                left.setTargetPosition(0);
                right.setTargetPosition(0);
                right.setPower(0.3);
                left.setPower(0.3);
            } else if (gamepad1.dpad_left) {
                servo1.setPosition(0.2);
            } else if (gamepad1.dpad_right) {
                servo1.setPosition(0.7);
            }
            //else {
            //    right.setPower(0.0);
            //    left.setPower(0.0);
            //}
            telemetry.addData("Current position", "left position: " + left.getCurrentPosition());
            telemetry.addData("Current position", "right position: " + right.getCurrentPosition());
            telemetry.update();
        }
    }
}