package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.archived23_24SeaonCenterStage.yiseArchived.LedLights;
import org.firstinspires.ftc.teamcode.yise.LiftClass;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;
import org.firstinspires.ftc.teamcode.yise.ledLights;

@TeleOp(name="Encoder Testing", group="Linear Opmode")
public class Encoder extends LinearOpMode {

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

    public Boolean color = false;


    public Boolean Hang = false;

    public Boolean RightBumperPressed = false;

    public Boolean buttonPressed = false;

    public Boolean Uptapped = false;
    public Boolean Downtapped = false;
    public Boolean Lefttapped = false;
    public Boolean Righttapped = false;

    public double wrist = 0;

    public boolean canToggleSlowMode = true;

    @Override
    public void runOpMode() throws InterruptedException {
        LiftClass arm = new LiftClass(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
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

        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {

            if (Parameters.allianceColor == Parameters.Color.RED && !color) {
            leds.setLed(ledLights.ledStates.RED);
            } else if (!color ) {
                leds.setLed(ledLights.ledStates.BLUE);
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double strafe =  gamepad1.left_stick_x;
            double turn     =  gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                forward = 0.2;
            } else if (gamepad1.dpad_down) {
                forward = -0.2;
            } else if (gamepad1.dpad_left) {
                strafe = -0.2;
            } else if (gamepad1.dpad_right) {
                strafe = 0.2;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = forward + strafe + turn;
            double rightFrontPower = -forward + strafe + turn;
            double leftBackPower   = -forward + strafe - turn;
            double rightBackPower  = -forward - strafe + turn;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);



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
                Uptapped = true;
            } else if (gamepad2.dpad_down) {
                Downtapped = true;
            } else if (gamepad2.dpad_left) {
                Lefttapped = true;
            } else if (gamepad2.dpad_right) {
                Righttapped = true;
            }

            if (gamepad1.touchpad || gamepad1.ps|| gamepad1.start) {
                Hang = true;
            }
                if (Hang) {
                    switch (state) {
                        case -1:  // Initialize the
                            arm.setShoulderPosition(0.25);
                            arm.setElbowPosition(0);
                            state = 0;
                            break;
                        case 0:  // Initialize the
                            arm.setPulleyPosition(LiftClass.PulleyPosition.HANG);
                            if (arm.getPulleyPositionL() >= 2400) {
                                state++;
                            }
                            break;
                        case 1:
                            arm.setLiftPosition(LiftClass.liftPosition.HANG);
                            if (arm.getLiftPositionL() >= 150) { // Replace with your own position checking logic
                                sleep(2000);
                                state++;
                            }
                            break;
                        case 2:
                            arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
                            if (arm.getPulleyPositionR() <= 150) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 3:
                            arm.setShoulderPosition(1);
                            arm.setElbowPosition(0.65);
                            state++;
                            break;

                        case 4:
                            Hang = true;
                            arm.setPulleyPosition(LiftClass.PulleyPosition.HANGEND);
                            if (arm.getPulleyPositionR() >= 600) { // Replace with your own position checking logic
                                sleep(2000);
                                state++;
                            }
                            break;
                        case 5:
                            arm.heroPowerPulley();
                    }
                }

                if (Uptapped) {

                    switch (state) {
                        case -1:  // Initialize the
                            arm.setShoulderPosition(0.25);
                            arm.setElbowPosition(0);
                            state = 0;
                            break;
                        case 0:  // Initialize the
                            if (!buttonPressed) {
                                arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
                                buttonPressed = true;
                            }
                            if (arm.getPulleyPositionL() <= 350) {
                                state++;
                            }
                            break;
                        case 1:
                            arm.setLiftPosition(LiftClass.liftPosition.BASKET);
                            if (arm.getLiftPositionL() >= 300) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 2:
                            arm.setPulleyPosition(LiftClass.PulleyPosition.BASKET);
                            if (arm.getPulleyPositionR() >= 3000) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 3:
                            arm.setShoulderPosition(1);
                            arm.setElbowPosition(0.65);
                            state = -1;
                            Uptapped = false;
                            break;
                    }
                } else if (Downtapped) {
                    switch (state) {
                        case -1:  // Initialize the
                            arm.setShoulderPosition(0.25);
                            arm.setElbowPosition(0);
                            state = 0;
                            break;
                        case 0:  // Initialize the
                            if (!buttonPressed) {
                                arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
                                buttonPressed = true;
                            }
                            if (arm.getPulleyPositionL() <= 350) {
                                state++;
                            }
                            break;
                        case 1:
                            arm.setLiftPosition(LiftClass.liftPosition.HOME);
                            if (arm.getLiftPositionL() <= 100) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 2:
                            arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
                            if (arm.getPulleyPositionR() <= 400) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 3:
                            arm.setShoulderPosition(0.25);
                            arm.setElbowPosition(0);
                            state = -1;
                            Downtapped = false;
                            break;
                    }
                } else if (Lefttapped) {

                    switch (state) {
                        case -1:  // Initialize the
                            arm.setShoulderPosition(0.25);
                            arm.setElbowPosition(0);
                            state = 0;
                            break;
                        case 0:  // Initialize the
                            if (!buttonPressed) {
                                arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
                                buttonPressed = true;
                            }
                            if (arm.getPulleyPositionL() <= 350) {
                                state++;
                            }
                            break;
                        case 1:
                            arm.setLiftPosition(LiftClass.liftPosition.HOME);
                            if (arm.getLiftPositionL() <= 100) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 2:
                            arm.setPulleyPosition(LiftClass.PulleyPosition.SEARCH);
                            if (arm.getPulleyPositionR() >= 1500) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 3:
                            arm.setShoulderPosition(0.4);
                            arm.setElbowPosition(0);
                            state = -1;
                            Lefttapped = false;
                            break;
                    }
                } else if (Righttapped) {

                    switch (state) {
                        case -1:  // Initialize the
                            arm.setShoulderPosition(0.25);
                            arm.setElbowPosition(0);
                            state = 0;
                            break;
                        case 0:  // Initialize the
                            if (!buttonPressed) {
                                arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
                                buttonPressed = true;
                            }
                            if (arm.getPulleyPositionL() <= 350) {
                                state++;
                            }
                            break;
                        case 1:
                            arm.setLiftPosition(LiftClass.liftPosition.SUBMERSABLE);
                            if (arm.getLiftPositionL() >= 100) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 2:
                            arm.setPulleyPosition(LiftClass.PulleyPosition.SUBMERSABLE);
                            if (arm.getPulleyPositionR() >= 1500) { // Replace with your own position checking logic
                                state++;
                            }
                            break;
                        case 3:
                            arm.setShoulderPosition(0.4);
                            arm.setElbowPosition(0.125);
                            state = -1;
                            Righttapped = false;
                            break;
                    }
                } else if (gamepad2.options) {
                    arm.manualPowerDownPulley();
                } else if (gamepad2.touchpad) {
                    //arm.manualPowerUpPulley();
                } else if (arm.getCurrentPulleyPosition() == LiftClass.PulleyPosition.BASKET || arm.getCurrentPulleyPosition() == LiftClass.PulleyPosition.SUBMERSABLE) {
                    arm.zeroPowerPulley();
                }

                if (!gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right) {
                    buttonPressed = false;
                }

                if (gamepad2.y) {
                    arm.setLiftPosition(LiftClass.liftPosition.HOME);
                } else if (arm.getCurrentLiftPosition() == LiftClass.liftPosition.BASKET) {
                    arm.zeroPowerLift();
                }

                // Claw control method
                if (gamepad1.right_bumper || gamepad2.right_trigger > 0.15 && !RightTriggerPressed) {
                    RightTriggerPressed = true;
                    if (arm.claw.getPosition() == 1) {
                        arm.claw.setPosition(0);
                        ;
                    } else {
                        arm.claw.setPosition(1);
                    }
                } else if (gamepad2.right_trigger < 0.15 && !gamepad1.right_bumper) {
                    RightTriggerPressed = false;
                }

                if (gamepad2.b) {
                    arm.setShoulderPosition(0.675);
                    arm.setElbowPosition(0.2);
                } else if (gamepad2.x) {
                    arm.setShoulderPosition(0.65);
                    arm.setElbowPosition(0.5);
                }

                if (gamepad1.right_bumper || gamepad2.right_bumper && !RightBumperPressed) {
                    RightBumperPressed = true;
                    if (arm.wrist.getPosition() == 0) {
                        arm.wrist.setPosition(0.33);
                    } else if (arm.wrist.getPosition() == 0.33) {
                        arm.wrist.setPosition(0.66);
                    } else if (arm.wrist.getPosition() == 0.66) {
                        arm.wrist.setPosition(0.99);
                    } else {
                        arm.wrist.setPosition(0);
                    }
                } else if (!gamepad2.right_bumper && RightBumperPressed) {
                    RightBumperPressed = false;
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

                telemetry.addData("button pressed", buttonPressed);

                telemetry.addLine();

            telemetry.addData("hang", Hang);

            telemetry.addData("colorR", clawSensor.red());
                telemetry.addData("colorB", clawSensor.blue());
                telemetry.addData("colorG", clawSensor.green());

                telemetry.update();
            }
        }
    }