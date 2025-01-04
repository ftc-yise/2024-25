package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Game Values (RUN THIS EVERY MATCH)", group="Necessity")
public class Parameters extends LinearOpMode {

    public enum AutonomousConfig {
        BASKET,
        OBSERVATION
    }

    public enum EndingPosition {
        ACCENT,
        OBSERVATION
    }

    public enum Color {
        RED,
        BLUE
    }

    public static AutonomousConfig autoConfig;
    public static EndingPosition endingPosition;
    public static Color allianceColor;

    public static double WAIT = 0;
    public boolean xReleased;

    @Override
    public void runOpMode() {

        while (!gamepad1.a && !gamepad1.b) {
            telemetry.addLine("Alliance Color \n");
            telemetry.addLine("X - Blue \n O - Red");

            telemetry.update();

            if (gamepad1.a) {
                allianceColor = Color.BLUE;
            } else if (gamepad1.b) {
                allianceColor = Color.RED;
            }
        }


        while (gamepad1.a || gamepad1.b) {
            //WAIT until released
        }


        while (!gamepad1.a && !gamepad1.b) {
            telemetry.addLine("Starting Position \n");
            telemetry.addLine("X - BASKET \n O - Observation Zone");

            telemetry.update();

            if (gamepad1.a) {
                    autoConfig = AutonomousConfig.BASKET;
            } else if (gamepad1.b) {
                autoConfig = AutonomousConfig.OBSERVATION;
            }
        }

        while (gamepad1.a || gamepad1.b) {
            //Wait until released
        }

        while (!gamepad1.a && !gamepad1.b && !gamepad1.x) {
            telemetry.addLine("Park Position \n");
            telemetry.addLine("▢ - Accent Level 1 \n X - Observation Zone");

            telemetry.update();

            if (gamepad1.x) {
                endingPosition = EndingPosition.ACCENT;
            } else if (gamepad1.a) {
                endingPosition = EndingPosition.OBSERVATION;
            }
        }

        while (gamepad1.a || gamepad1.b || gamepad1.x) {
            //Wait until released
        }

        while (!gamepad1.y) {
            telemetry.addLine("Wait Seconds: " + WAIT);
            telemetry.addLine("▢ = -1 \n X = +1 \n O = 0 \n Y to continue");
            telemetry.update();

            if (gamepad1.x && xReleased) {
                WAIT--;
                xReleased = false;
            } else if (gamepad1.a && xReleased) {
                WAIT++;
                xReleased = false;
            } else if (gamepad1.b && xReleased) {
                WAIT = 0;
                xReleased = false;
            }

            if (!gamepad1.x && !gamepad1.a && !gamepad1.b && !xReleased){
                xReleased = true;
            }
        }

        while (gamepad1.y) {
            //Wait until released
        }

        while (!gamepad1.a) {
            telemetry.addLine("Color: " + allianceColor);
            telemetry.addLine("Starting Position: " + autoConfig);
            telemetry.addLine("Park position: " + endingPosition);
            telemetry.addLine("WAIT: " + WAIT);
            telemetry.addLine("\nX to end program");

            telemetry.update();
        }

        telemetry.addLine("Configuration complete. Self-destructing");
        telemetry.update();

        sleep(3000);

    }}
