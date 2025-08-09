package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.yise.exParams;
import org.firstinspires.ftc.teamcode.yise.exLedLights;

@TeleOp(name="Example Drive", group="Linear OpMode")
public class ExampleDrive extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public RevColorSensorV3 clawSensor;
    public Boolean colorChanged = false;

    @Override
    public void runOpMode() throws InterruptedException {
        exLedLights LEDs = new exLedLights(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LEDs.setLed(exLedLights.ledStates.INIT);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // checks for which color should be our default color based on our alliance
            if (exParams.allianceColor == exParams.Color.RED && (!colorChanged)) {
                LEDs.setLed(exLedLights.ledStates.RED);
            } else if (!colorChanged) {
                LEDs.setLed(exLedLights.ledStates.BLUE);
            }

            // set LED color based on color sensor in claw (i.e. what color block did we grab)
            if (clawSensor.green() > 650) {
                colorChanged = true;
                LEDs.setLed(exLedLights.ledStates.GRAB_Y);
            } else if (clawSensor.red() > 400) {
                colorChanged = true;
                LEDs.setLed(exLedLights.ledStates.GRAB_R);
            } else if (clawSensor.blue() > 400) {
                LEDs.setLed(exLedLights.ledStates.GRAB_B);
                colorChanged = true;
            } else {
                colorChanged = false;
            }
        }
    }