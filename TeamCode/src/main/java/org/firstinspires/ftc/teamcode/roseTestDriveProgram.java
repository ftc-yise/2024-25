package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.LedLights;


@TeleOp(name="Competition drive", group="Linear Opmode")
public class roseTestDriveProgram extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;
    boolean reverseIntake = false;
    boolean manualSlide = false;
    boolean manualHand = false;

    @Override
    public void runOpMode() {

        LedLights leds = new LedLights(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if (Parameters.allianceColor == Parameters.Color.RED) {
            leds.setLed(LedLights.ledStates.RED);
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            leds.setLed(LedLights.ledStates.BLUE);
        }

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                rrDrive.updateFromDpad(-0.2, 0, 0);
            } else if (gamepad1.dpad_up) {
                rrDrive.updateFromDpad(0.2, 0, 0);
            } else if (gamepad1.dpad_left) {
                rrDrive.updateFromDpad(0, 0.2, 0);
            } else if (gamepad1.dpad_right) {
                rrDrive.updateFromDpad(0, -0.2, 0);
            } else {
                rrDrive.updateMotorsFromStick(gamepad1);
            }
            rrDrive.update();

            /**
             * Intake
             */
            //Check if both slots are full in bucket
            if (colorSensors.getBackPixelColor() != DriveColorExample.Colors.NONE && colorSensors.getFrontPixelColor() != DriveColorExample.Colors.NONE) {
                //Turn on yellow color, 1 pixel=
                reverseIntake = true;
                leds.setLed(LedLights.ledStates.GREEN);
            } else if (colorSensors.getBackPixelColor() != DriveColorExample.Colors.NONE) {
                leds.setLed(LedLights.ledStates.YELLOW);
                reverseIntake = false;
            } else {
                //Turn to alliance color
                if (getRuntime() > 85) {
                    leds.setLed(LedLights.ledStates.ENDGAME);
                } else {
                    if (Parameters.allianceColor == Parameters.Color.RED) {
                        leds.setLed(LedLights.ledStates.RED);
                    } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
                        leds.setLed(LedLights.ledStates.BLUE);
                    }
                }
                reverseIntake = false;
            }

            if ((gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5)) {
                //If both slot are full, outtake excess pixels
                if (reverseIntake) {
                    intakeSystem.runIntakeSystem(-1);
                    //leds.setLed(LedLights.ledStates.INTAKE);
                } else {
                    intakeSystem.runIntakeSystem(1);
                    //leds.setLed(LedLights.ledStates.INTAKE);
                }
            } else if ((gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5)) {
                //Manual outtake
                intakeSystem.runIntakeSystem(-0.5);
            } else {
                //If no inputs, stop intake
                intakeSystem.runIntakeSystem(0);
            }


            /**
             * Arm slides
             */
            //If driver input extend the slides to different legnths
            if (gamepad2.dpad_up) {
                arm.extend(LiftArm.Distance.FULL);
                arm.holdArm();
            } else if (gamepad2.dpad_right) {
                arm.extend(LiftArm.Distance.HALF);
                arm.holdArm();
            } else if (gamepad2.dpad_left) {
                arm.extend(LiftArm.Distance.LOW);
                arm.holdArm();
            } else if (gamepad2.dpad_down) {
                arm.retract();
            }

            if (gamepad2.left_stick_button){
                arm.manualArm(-1);
                manualSlide = true;
            } else if (manualSlide == true && !gamepad2.left_stick_button) {
                arm.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                manualSlide = false;
            }

            if (gamepad2.right_stick_button) {
                manualHand = true;
                arm.manualIn();
            } else if (!gamepad2.right_stick_button && manualHand == true) {
                manualHand = false;
                arm.hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.hand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            /**
             * Trapdoor
             */
            if (gamepad2.right_bumper) {
                arm.openTrapdoor();
            } else if (gamepad2.left_bumper && !trapdoorMoving) {
                trapdoorMoving = true;
                arm.openTrapdoor();
                sleep(74 );
                arm.closeTrapdoor();
            } else {
                arm.closeTrapdoor();
            }

            if (!gamepad2.left_bumper) {
                trapdoorMoving = false;
            }

            /**
             * Hang
             */
            if (gamepad1.a && gamepad1.x){
                arm.setArmDistance(LiftArm.Distance.ENDGAMESTART);
                armOut = true;
                leds.setLed(LedLights.ledStates.HANG);
            } else if (!gamepad1.a && armOut) {
                arm.setArmDistance(LiftArm.Distance.ENDGAMEHOLD);
                arm.holdHang();
                leds.setLed(LedLights.ledStates.CELEBRATION);
            }


            /**
             * Airplane
             */
            if (gamepad2.x && gamepad2.a) {
                arm.launchPlane();
            } else {
                arm.reloadPlane();
            }

            /**
             * Slow mode toggle
             */
            //If input released, slow mode can be toggled again. This prevents an infinite loop of toggling.
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;
                //Toggle between slow and normal speeds
                switch (rrDrive.currentSpeed) {
                    case SLOW:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.SLOW);
                        break;
                }
            }


            /**
             * Telemetry data
             */
            //telemetry.addData("Servo: ", arm.plane.getPosition());
            /*telemetry.addData("Slide: ", arm.getSlidePosition());
            telemetry.addData("Arm pos: ", arm.getHandPosition());
            telemetry.addData("Hand power: ", arm.hand.getPower());*/
            telemetry.addData("Servo: ", arm.purplePixel.getPosition());

            /*telemetry.addData("X: ", rrDrive.getPosition().getX());
            telemetry.addData("Y: ", rrDrive.getPosition().getY());
            telemetry.addData("Heading: ", rrDrive.getPosition().getHeading());*/

            telemetry.addLine();

            /*telemetry.addData("Red color front: ", colorSensors.getRedColor()[0]);
            telemetry.addData("Blue color front: ", colorSensors.getBlueColor()[0]);
            telemetry.addData("Green color front: ", colorSensors.getGreenColor()[0]);
            telemetry.addData("Red color back: ", colorSensors.getRedColor()[1]);
            telemetry.addData("Blue color back: ", colorSensors.getBlueColor()[1]);
            telemetry.addData("Green color back: ", colorSensors.getGreenColor()[1]);*/

            /*telemetry.addData("Back pixel color: ", colorSensors.getBackPixelColor());
            telemetry.addData("Front pixel color: ", colorSensors.getFrontPixelColor());*/
            /*telemetry.addData("Ratio back: ", (colorSensors.getRedColor()[1] + colorSensors.getGreenColor()[1])/colorSensors.getBlueColor()[1]);
            telemetry.addData("Ratio front: ", (colorSensors.getRedColor()[0] + colorSensors.getGreenColor()[0])/colorSensors.getBlueColor()[0]);*/

            telemetry.update();
        }
    }
}