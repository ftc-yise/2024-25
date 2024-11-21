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
               arm.setLiftPosition(liftArm.armPosition.DOWN);
            } else if (gamepad1.dpad_up) {
                arm.setLiftPosition(liftArm.armPosition.UP);
            } else if (gamepad1.left_bumper) {
                arm.manualPowerDownLift();
            }else if (gamepad1.right_bumper) {
                arm.manualPowerUpLift();
            } else {
                arm.zeroPowerLift();
            }

            if (gamepad1.right_trigger > 0.75) {
               arm.manualPowerUpPulley();
            } else if (gamepad1.left_trigger > 0.75) {
                arm.manualPowerDownPulley();
            }else if (gamepad1.dpad_left) {
                arm.setPulleyPosition(liftArm.PulleyPosition.IN);
            } else if (gamepad1.dpad_right) {
                arm.setPulleyPosition(liftArm.PulleyPosition.OUT);}
            else {
                arm.zeroPowerPulley();
            }

            telemetry.addData("Left Lift Encoder Position", arm.getLiftPositionL());
            telemetry.addData("Right Lift Encoder Position", arm.getLiftPositionR());

            telemetry.addData("Pulley Right", arm.getPulleyPositionR());
            telemetry.addData("Pulley Left", arm.getPulleyPositionL());

            telemetry.addData("Pulley PowerL", arm.PulleyPowerL());
            telemetry.addData("Pulley PowerR", arm.PulleyPowerR());
            
            telemetry.update();
        }
    }
}