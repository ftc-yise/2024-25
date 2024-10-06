package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.ledLights;


@TeleOp(name="Competition drive", group="Linear Opmode")
public class roseTestDriveProgram extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        ledLights leds = new ledLights(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
               leds.setLed(ledLights.ledStates.BLOCK_GRABBED);
            } else if (gamepad1.dpad_up) {
                leds.setLed(ledLights.ledStates.ARM_READY);
            } else if (gamepad1.dpad_left) {
                leds.setLed(ledLights.ledStates.ENDGAME);
            } else if (gamepad1.dpad_right) {
                leds.setLed(ledLights.ledStates.INIT);
            }


            /**
             * Telemetry data
             */

            telemetry.addData("currentstate: ", leds.currentState);

            telemetry.update();
        }
    }
}