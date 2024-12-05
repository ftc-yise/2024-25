package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.liftArm;

@TeleOp(name="jack Motor Testing", group="Linear Opmode")
public class jackMotorTesting extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        liftArm arm = new liftArm(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
               arm.setLiftPosition(liftArm.liftPosition.DOWN);
            } else if (gamepad1.dpad_up) {
                arm.setLiftPosition(liftArm.liftPosition.UP);
            }
            telemetry.addData("Left Encoder Position", arm.leftLiftMotorPositionValue);
            telemetry.addData("Right Encoder Position", arm.rightLiftMotorPositionValue);
            telemetry.update();
        }
    }
}