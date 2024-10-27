package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.yise.liftArm;

@TeleOp(name="jack Motor Testing", group="Linear Opmode")
public class jackMotorTesting extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    public Servo servo1;

    @Override
    public void runOpMode() {
        liftArm arm = new liftArm(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        servo1 = hardwareMap.get(Servo.class, "servo1");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Servo position", "servo1 position: " + servo1.getPosition());

            if (gamepad1.dpad_down) {
               arm.setArmPosition(liftArm.armPosition.DOWN);
            } else if (gamepad1.dpad_up) {
                arm.setArmPosition(liftArm.armPosition.UP);
            } else if (gamepad1.dpad_left) {
                servo1.setPosition(0.2);
            } else if (gamepad1.dpad_right) {
                servo1.setPosition(0.7);
            }
            telemetry.update();
        }
    }
}