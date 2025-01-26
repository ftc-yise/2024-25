package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.yise.LiftClass;
import org.firstinspires.ftc.teamcode.yise.OpenCVVision;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;
import org.firstinspires.ftc.teamcode.yise.ledLights;

@TeleOp(name="Main Drive", group="Linear OpMode")
public class MainDrive extends LinearOpMode {
    OpenCVVision vision = new OpenCVVision();

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new  ElapsedTime();

    public RevColorSensorV3 clawSensor;

    public Boolean RightBumperPressed = false;
    public Boolean XPressed = false;
    public Boolean BPressed = false;
    public Boolean RightTriggerPressed = false;
    public Boolean BumperPressed = false;

    public Boolean color = false;

    public boolean canToggleSlowMode = true;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCVVision vision = new OpenCVVision(hardwareMap);

        LiftClass arm = new LiftClass(hardwareMap);

        RoadRunnerDriving drive = new RoadRunnerDriving(hardwareMap);

        ledLights LEDs = new ledLights(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        clawSensor = hardwareMap.get(RevColorSensorV3.class, "ClawSensor");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LEDs.setLed(ledLights.ledStates.INIT);

        // set which pipeline is used in INIT to check to check what it is seeing
        if (Parameters.vision == Parameters.Vision.RED){
            vision.setCameraPipeline(OpenCVVision.Color.RED);
        } else if (Parameters.vision == Parameters.Vision.BLUE) {
            vision.setCameraPipeline(OpenCVVision.Color.BLUE);
        } else if (Parameters.vision == Parameters.Vision.YELLOW) {
            vision.setCameraPipeline(OpenCVVision.Color.YELLOW);
        } else {
            vision.setCameraPipeline(OpenCVVision.Color.TEST);
        }

        telemetry.addData("Color:", vision.getColor());
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //checks for which color should be our "Default" color
            // based on what team color where on
            if (Parameters.allianceColor == Parameters.Color.RED && !color) {
                LEDs.setLed(ledLights.ledStates.RED);
            } else if (!color) {
                LEDs.setLed(ledLights.ledStates.BLUE);
            }

            if (gamepad1.dpad_down) {
                drive.updateFromDpad(-0.2, 0, 0);
            } else if (gamepad1.dpad_up) {
                drive.updateFromDpad(0.2, 0, 0);
            } else if (gamepad1.dpad_left) {
                drive.updateFromDpad(0, 0.2, 0);
            } else if (gamepad1.dpad_right) {
                drive.updateFromDpad(0, -0.2, 0);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }
            drive.update();

            // toggles on and off our slow-mode fast-mode to give
            // drivers speedy yet concise movement
            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;
                //Toggle between slow and normal speeds
                switch (drive.currentSpeed) {
                    case SLOW:
                        drive.toggleSlowMode(RoadRunnerDriving.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        drive.toggleSlowMode(RoadRunnerDriving.Speeds.SLOW);
                        break;
                }
            }

            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            // arm code which both defines if our arm is moving and
            // which dpad we hit
            if (gamepad1.touchpad || gamepad1.ps) {
                arm.currentMovementState = LiftClass.movementState.MOVING;
                arm.currentButtonPressedState = LiftClass.buttonPressedState.HANG;

            } else if (gamepad2.dpad_up) {
                arm.currentMovementState = LiftClass.movementState.MOVING;
                arm.currentButtonPressedState = LiftClass.buttonPressedState.DPAD_UP;

            } else if (gamepad2.dpad_down) {
                arm.currentMovementState = LiftClass.movementState.MOVING;
                arm.currentButtonPressedState = LiftClass.buttonPressedState.DPAD_DOWN;

            } else if (gamepad2.dpad_left) {
                arm.currentMovementState = LiftClass.movementState.MOVING;
                arm.currentButtonPressedState = LiftClass.buttonPressedState.DPAD_LEFT;

            } else if (gamepad2.dpad_right) {
                arm.currentMovementState = LiftClass.movementState.MOVING;
                arm.currentButtonPressedState = LiftClass.buttonPressedState.DPAD_RIGHT;
            } else if (gamepad2.options) {
                arm.manualPowerDownPulley();
            } else if (gamepad2.left_trigger > 0.75){
                arm.manualPowerDownLift();
            } else if (arm.limit.getState() && !arm.buttonPressed){
                //arm.currentMovementState = LiftClass.movementState.REST;
                //arm.currentHoldPowerState = LiftClass.holdPowerState.HOME;
            }

            if (gamepad2.left_trigger > 0.75){
                arm.manualPowerDownLift();
            }

            if (!gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right) {
                arm.setButtonPressedStatus(false);
            } else {
                arm.setButtonPressedStatus(false);
            }

            //set whole are movement based on which dpad is pressed and if we need
            //to manually in
            switch (arm.currentButtonPressedState){
                case HANG:
                    arm.setArmPosition(LiftClass.armPosition.HANG);
                    break;
                case DPAD_UP:
                    arm.setArmPosition(LiftClass.armPosition.BASKET);
                    break;
                case DPAD_DOWN:
                    if (arm.getSubmersibleScoringPosition()) {
                        arm.setArmPosition(LiftClass.armPosition.SUBMERSIBLEEND);
                    } else{
                        arm.setArmPosition(LiftClass.armPosition.HOME);
                    }
                    break;
                case DPAD_LEFT:
                    arm.setArmPosition(LiftClass.armPosition.SEARCH);
                    break;
                case DPAD_RIGHT:
                    if (arm.getSubmersibleScoringPosition()) {
                        arm.setArmPosition(LiftClass.armPosition.SUBMERSIBLEEND);
                    } else {
                        arm.setArmPosition(LiftClass.armPosition.SUBMERSIBLESTART);
                    }
                    break;
                case REST:
                    telemetry.addLine("No buttons pressed");
                    break;
            }

            // Check arm movement state and adjust hold power accordingly
            // If the arm is MOVING, it will output "Moving".
            // If the arm is in a REST state, it checks the current hold power state
            // and applies corresponding actions for each case (e.g., Basket, Home, Hang).
            switch (arm.currentMovementState) {
                case MOVING:
                    telemetry.addLine("Moving");
                    break;
                case REST:
                    switch (arm.currentHoldPowerState) {
                        case BASKET:
                            telemetry.addLine("Basket");
                            arm.zeroPowerLift();
                            arm.zeroPowerPulley();
                            break;
                        case HOME:
                            telemetry.addLine("Home");
                            arm.neutralPowerLift();
                            arm.neutralPowerPulley();
                            break;
                        case HANG:
                            arm.hangPowerLift();
                            arm.hangPowerPulley();
                            telemetry.addLine("Hang");
                            break;
                        case SUBMERSIBLE:
                            arm.zeroPowerLift();
                            arm.neutralPowerPulley();
                            telemetry.addLine("Submersible");
                            break;
                        case SEARCH:
                            arm.neutralPowerLift();
                            arm.neutralPowerPulley();
                            telemetry.addLine("Submersible");
                            break;
                    }
                    break;
            }

            // Claw control method
            // uses a ternary operation condition ? valueIfTrue : valueIfFalse
            // a ternary operator is a shortcut for an if statement
            if (gamepad2.right_trigger > 0.15 && !RightTriggerPressed) {
                RightTriggerPressed = true;
                arm.claw.setPosition(arm.claw.getPosition() == 1 ? 0 : 1);
            } else if (gamepad2.right_trigger < 0.15) {
                RightTriggerPressed = false;
            }

            // LED code for if we open our code
            if (arm.claw.getPosition() == 1){
                LEDs.setLed(ledLights.ledStates.CLAW_OPEN);
            }

            // this is our toggle to move servos into ground search positions
            if (gamepad2.b && !BPressed) {
                BPressed = true;
                if (arm.elbow.getPosition() == 0.02){
                    arm.setShoulderPosition(0.4);
                    arm.setElbowPosition(0);
                } else {
                    arm.setShoulderPosition(0.6);
                    arm.setElbowPosition(0.02);
                }
            }
            else if (!gamepad2.b && BPressed) {
                BPressed = false;
            }

            //controlling shoulder when grabbing specimen of the wall position using a
            // toggle boolean and a ternary operator
            if (gamepad2.x && !XPressed) {
                XPressed = true;

                 arm.setShoulderPosition(arm.ShoulderR.getPosition() == 0.25 ? 1 : 0.25);

                arm.setElbowPosition(0);
            } else if (!gamepad2.x && XPressed) {
                XPressed = false;
            }

            //controlling wrist position using a toggle boolean and a ternary operator
            // Set wrist position: if current position is 1, set to 0;
            // if current position is 0, set to 0.5;
            // otherwise, set to 1.
            if (gamepad2.right_bumper && !RightBumperPressed) {
                RightBumperPressed = true;

                arm.wrist.setPosition(arm.wrist.getPosition() == 1 ? 0 : (arm.wrist.getPosition() == 0 ? 0.5 : 1));

            } else if (!gamepad2.right_bumper && RightBumperPressed) {
                RightBumperPressed = false;
            }

            // this if else statement controls both OpenCV pipelines and camera heights
            if (gamepad1.right_bumper){
                arm.setCameraHeightHIGH();
                if (Parameters.allianceColor == Parameters.Color.RED){
                    vision.setCameraPipeline(OpenCVVision.Color.RED);
                } else {
                    vision.setCameraPipeline(OpenCVVision.Color.BLUE);
                }
            } else if (gamepad1.left_bumper && !BumperPressed) {
                arm.setCameraHeightLOW();
                BumperPressed = true;
                if (vision.getColor() == OpenCVVision.Color.YELLOW){
                    if (Parameters.allianceColor == Parameters.Color.RED){
                        vision.setCameraPipeline(OpenCVVision.Color.RED);
                    } else {
                        vision.setCameraPipeline(OpenCVVision.Color.BLUE);
                    }
                } else {
                    vision.setCameraPipeline(OpenCVVision.Color.YELLOW);
                }
            }
            if (!gamepad1.left_bumper){
                BumperPressed = false;
            }

            //LED code based on the color sensor to decide what color our LEDs should
            //display
            if (clawSensor.green() > 650) {
                color = true;
                LEDs.setLed(ledLights.ledStates.GRAB_Y);
            } else if (clawSensor.red() > 400) {
                color = true;
                LEDs.setLed(ledLights.ledStates.GRAB_R);
            } else if (clawSensor.blue() > 400) {
                LEDs.setLed(ledLights.ledStates.GRAB_B);
                color = true;
            } else {
                color = false;
            }


            // Telemetry Segments
            // The telemetry output is grouped into 7 sections for clarity:
            // 1. Encoder positions
            // 2. Motor powers
            // 3. Current arm movement (Enum values)
            // 4. Servo positions
            // 5. Color sensor values
            // 6. Arm movement control booleans
            // 7. D-pad control booleans for arm movement

            // Section 1: Encoder positions
            telemetry.addData("Pulley Right", arm.getPulleyPositionR());
            telemetry.addData("Pulley Left", arm.getPulleyPositionL());
            telemetry.addData("Left Lift Encoder Position", arm.getLiftPositionL());
            telemetry.addData("Right Lift Encoder Position", arm.getLiftPositionR());
            telemetry.addLine();

            // Section 2: Motor powers
            telemetry.addData("Pulley PowerL", arm.PulleyPowerL());
            telemetry.addData("Pulley PowerR", arm.PulleyPowerR());
            telemetry.addData("Lift PowerL", arm.LiftPowerL());
            telemetry.addData("Lift PowerR", arm.LiftPowerR());
            telemetry.addLine();

            // Section 3: Current arm movement (Enum values)
            telemetry.addData("pulleyPose", arm.getCurrentPulleyPosition());
            telemetry.addData("liftPose", arm.getCurrentLiftPosition());
            telemetry.addData("armMovement", arm.getCurrentArmMovement());
            telemetry.addLine();

            // Section 4: Servo positions
            telemetry.addData("shoulder", arm.ShoulderR.getPosition());
            telemetry.addData("elbow", arm.elbow.getPosition());
            telemetry.addData("claw", arm.claw.getPosition());
            telemetry.addData("wrist", arm.wrist.getPosition());
            telemetry.addLine();

            // Section 5: Color sensor values
            telemetry.addData("colorR", clawSensor.red());
            telemetry.addData("colorB", clawSensor.blue());
            telemetry.addData("colorG", clawSensor.green());
            telemetry.addLine();

            // Section 6: Arm movement control booleans
            telemetry.addData("targetPose", arm.pulleyLeft.getTargetPosition());
            telemetry.addData("button pressed", arm.getButtonPressed());
            telemetry.addLine();

            // Section 7: Limit Switch
            telemetry.addData("limit switch", arm.limit.getState());
            telemetry.addLine();

            telemetry.update();
        }
    }
}