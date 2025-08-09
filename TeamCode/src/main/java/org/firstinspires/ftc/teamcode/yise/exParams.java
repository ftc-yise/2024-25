package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Game Values (RUN THIS EVERY MATCH)", group="Necessity")
public class exParams extends LinearOpMode {

    public enum Color {
        RED,
        BLUE
    }

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

        while (!gamepad1.a) {
            telemetry.addLine("Color: " + allianceColor);
            telemetry.addLine("\nX to end program");
            telemetry.update();
        }

        telemetry.addLine("Configuration complete. Self-destructing");
        telemetry.update();

        sleep(3000);

    }}
