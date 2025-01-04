package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.yise.LiftClass;
import org.firstinspires.ftc.teamcode.yise.OpenCVVision;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;
import org.firstinspires.ftc.teamcode.yise.ledLights;

@TeleOp(name="Encoder Testing", group="Linear Opmode")
public class Encoder extends LinearOpMode {
    OpenCVVision vision = new OpenCVVision();

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public RevColorSensorV3 clawSensor;

    // Declare a state variable
    int state = -1;

    public Boolean RightTriggerPressed = false;

    public Boolean BumberPressed = false;

    public Boolean color = false;

    public Boolean RightBumperPressed = false;
    public Boolean XPressed = false;
    public Boolean BPressed = false;

    public boolean canToggleSlowMode = true;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCVVision vision = new OpenCVVision(hardwareMap);

        LiftClass arm = new LiftClass(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        clawSensor = hardwareMap.get(RevColorSensorV3.class, "ClawSensor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        RoadRunnerDriving drive = new RoadRunnerDriving(hardwareMap);
        ledLights leds = new ledLights(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leds.setLed(ledLights.ledStates.INIT);

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

            if (Parameters.allianceColor == Parameters.Color.RED && !color) {
                leds.setLed(ledLights.ledStates.RED);
            } else if (!color) {
                leds.setLed(ledLights.ledStates.BLUE);
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

            if (gamepad2.dpad_up) {
                arm.setDpadUpTappedStatus(true);
            } else if (gamepad2.dpad_down) {
                arm.setDpadDowntappedStatus(true);
            } else if (gamepad2.dpad_left) {
                arm.setDpadLefttappedStatus(true);
            } else if (gamepad2.dpad_right) {
                arm.setDpadRighttappedStatus(true);
            }

            if (gamepad1.touchpad || gamepad1.ps || gamepad1.start) {
                arm.setHangStatus(true);
            }
            if (arm.getHangStatus()) {
                arm.setArmPosition(LiftClass.armPosition.HANG);
            }

            if (arm.getDpadUpTapped()) {
                arm.setArmPosition(LiftClass.armPosition.BASKET);

            } else if (arm.getDpadDownTapped()) {
                if (arm.getCurrentArmMovement() == LiftClass.armPosition.SUBMERSIBLESTART && !arm.getButtonPressed()) {
                    arm.setArmPosition(LiftClass.armPosition.SUBMERSIBLEEND);

                } else if (!arm.getButtonPressed()){
                    arm.setArmPosition(LiftClass.armPosition.HOME);

                }
            } else if (arm.getDpadLeftTapped()) {
                arm.setArmPosition(LiftClass.armPosition.SEARCH);

            } else if (arm.getDpadRightTapped()) {
                if (arm.getCurrentArmMovement() == LiftClass.armPosition.SUBMERSIBLESTART && !arm.getButtonPressed()) {
                    arm.setArmPosition(LiftClass.armPosition.SUBMERSIBLEEND);
                } else if (!arm.getButtonPressed()) {
                    arm.setArmPosition(LiftClass.armPosition.SUBMERSIBLESTART);

                }

            } else if (gamepad2.options) {
                arm.manualPowerDownPulley();
            } else if (arm.getCurrentPulleyPosition() == LiftClass.PulleyPosition.BASKET || arm.getCurrentPulleyPosition() == LiftClass.PulleyPosition.SUBMERSIBLE && arm.getPulleyHoldStatus()) {
                arm.zeroPowerPulley();
            } else if (Math.abs(arm.pulleyLeft.getCurrentPosition() - arm.pulleyLeft.getTargetPosition()) <= 250 && !arm.getPulleyHoldStatus()){
                arm.setPulleyPower(0);
                arm.setPulleyHoldStatus(false);
            }

            if (!gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right) {
                arm.setButtonPressedStatus(false);
            } else {
                arm.setPulleyHoldStatus(false);
            }

            if (arm.getCurrentLiftPosition() == LiftClass.liftPosition.BASKET) {
                arm.zeroPowerLift();
            }


            // Claw control method
            if (gamepad2.right_trigger > 0.15 && !RightTriggerPressed) {
                RightTriggerPressed = true;
                if (arm.claw.getPosition() == 1) {
                    arm.claw.setPosition(0);
                } else {
                    arm.claw.setPosition(1);
                }
            } else if (gamepad2.right_trigger < 0.15 && !gamepad1.right_bumper) {
                RightTriggerPressed = false;
            }

            if (arm.claw.getPosition() == 1){
                leds.setLed(ledLights.ledStates.CLAW_OPEN);
            }

            if (gamepad2.right_trigger > 0.75) {
                arm.setIntakePower(1);
            } else if (gamepad2.left_trigger > 0.75) {
                arm.setIntakePower(-1);
            } else {
                arm.setIntakePower(0);
            }


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


            if (gamepad2.x && !XPressed) {
                XPressed = true;
                if (arm.ShoulderR.getPosition() == 0.65) {
                    arm.setShoulderPosition(0.45);
                } else {
                    arm.setShoulderPosition(0.65);
                    arm.setElbowPosition(0.5);
                }
            } else if (!gamepad2.x && XPressed) {
                XPressed = false;
            }

            if (gamepad2.right_bumper && !RightBumperPressed) {
                RightBumperPressed = true;
                if (arm.wrist.getPosition() == 0) {
                    arm.wrist.setPosition(0.5);
                } else if (arm.wrist.getPosition() == 0.5) {
                    arm.wrist.setPosition(1);
                } else {
                    arm.wrist.setPosition(0);
                }
            } else if (!gamepad2.right_bumper && RightBumperPressed) {
                RightBumperPressed = false;
            }

            if (gamepad1.right_bumper){
                arm.setCameraHeightHIGH();
                if (Parameters.allianceColor == Parameters.Color.RED){
                    vision.setCameraPipeline(OpenCVVision.Color.RED);
                } else {
                    vision.setCameraPipeline(OpenCVVision.Color.BLUE);
                }
            } else if (gamepad1.left_bumper && !BumberPressed) {
                arm.setCameraHeightLOW();
                BumberPressed = true;
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
                BumberPressed = false;
            }

            if (clawSensor.green() > 650) {
                color = true;
                leds.setLed(ledLights.ledStates.GRAB_Y);
            } else if (clawSensor.red() > 400) {
                color = true;
                leds.setLed(ledLights.ledStates.GRAB_R);
            } else if (clawSensor.blue() > 400) {
                leds.setLed(ledLights.ledStates.GRAB_B);
                color = true;
            } else {
                color = false;
            }

            telemetry.addData("Left Lift Encoder Position", arm.getLiftPositionL());
            telemetry.addData("Right Lift Encoder Position", arm.getLiftPositionR());

            telemetry.addData("Pulley Right", arm.getPulleyPositionR());
            telemetry.addData("Pulley Left", arm.getPulleyPositionL());

            telemetry.addLine();

            telemetry.addData("Pulley PowerL", arm.PulleyPowerL());
            telemetry.addData("Pulley PowerR", arm.PulleyPowerR());

            telemetry.addData("Lift PowerL", arm.LiftPowerL());
            telemetry.addData("Lift PowerR", arm.LiftPowerR());

            telemetry.addLine();
            telemetry.addData("pullyPose", arm.getCurrentPulleyPosition());
            telemetry.addData("liftPose", arm.getCurrentLiftPosition());
            telemetry.addLine();

            telemetry.addData("shoulder", arm.ShoulderR.getPosition());
            telemetry.addData("elbow", arm.elbow.getPosition());
            telemetry.addData("claw", arm.claw.getPosition());
            telemetry.addData("wrist", arm.wrist.getPosition());

            telemetry.addData("button pressed", arm.getButtonPressed());

            telemetry.addLine();

            telemetry.addData("hang", arm.getHangStatus());

            telemetry.addData("colorR", clawSensor.red());
            telemetry.addData("colorB", clawSensor.blue());
            telemetry.addData("colorG", clawSensor.green());
            telemetry.addLine();
            telemetry.addData("pulleyhold", arm.getPulleyHoldStatus());
            telemetry.addData("taregtpose", arm.pulleyLeft.getTargetPosition());

            telemetry.addLine();
            telemetry.addData("armmovement", arm.getCurrentArmMovement());

            telemetry.update();
        }
    }
}