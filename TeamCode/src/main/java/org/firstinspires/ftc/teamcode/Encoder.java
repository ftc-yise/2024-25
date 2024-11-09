package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.liftArm;

@TeleOp(name="Encoder Testing", group="Linear Opmode")
public class Encoder extends LinearOpMode {

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
               arm.setArmPosition(liftArm.armPosition.DOWN);
            } else if (gamepad1.dpad_up) {
                arm.setArmPosition(liftArm.armPosition.UP);
            } else if (gamepad1.left_bumper) {
                arm.manualPowerDown();
            }else if (gamepad1.right_bumper) {
                arm.manualPowerUp();
            } else {
                arm.zeroPower();
            }
            telemetry.addData("Left Encoder Position", arm.leftArmMotorPositionValue);
            telemetry.addData("Right Encoder Position", arm.rightArmMotorPositionValue);
            telemetry.update();
        }
    }
}