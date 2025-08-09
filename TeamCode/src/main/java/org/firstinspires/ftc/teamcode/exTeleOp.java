package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.exParams;
import org.firstinspires.ftc.teamcode.yise.exLeds;
import org.firstinspires.ftc.teamcode.yise.exDrive;

@TeleOp(name="Example Teleop", group="Linear OpMode")
public class exTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public RevColorSensorV3 clawSensor;
    public Boolean colorChanged = false;
    public boolean canToggleSlowMode = true;

    @Override
    public void runOpMode() throws InterruptedException {
        exLeds LEDs = new exLeds(hardwareMap);
        exDrive drive = new exDrive(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LEDs.setLed(exLeds.ledStates.INIT);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //
            // LED related code
            //

            // checks for which color should be our default color based on our alliance
            if (exParams.allianceColor == exParams.Color.RED && (!colorChanged)) {
                LEDs.setLed(exLeds.ledStates.RED);
            } else if (!colorChanged) {
                LEDs.setLed(exLeds.ledStates.BLUE);
            }

            // set LED color based on color sensor in claw (i.e. what color block did we grab)
            if (clawSensor.green() > 650) {
                colorChanged = true;
                LEDs.setLed(exLeds.ledStates.GRAB_Y);
            } else if (clawSensor.red() > 400) {
                colorChanged = true;
                LEDs.setLed(exLeds.ledStates.GRAB_R);
            } else if (clawSensor.blue() > 400) {
                LEDs.setLed(exLeds.ledStates.GRAB_B);
                colorChanged = true;
            } else {
                colorChanged = false;
            }

            //
            // Drive related code
            //

            // update drive motor speeds/direction based on stick input
            drive.updateMotorsFromStick(gamepad1);
            drive.update();

            // toggles "slow mode" off and on
            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;
                switch (drive.currentSpeed) {
                    case SLOW:
                        drive.toggleSlowMode(exDrive.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        drive.toggleSlowMode(exDrive.Speeds.SLOW);
                        break;
                }
            }
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }
        }
    }
}