package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.yise.LiftClass;

@TeleOp(name="Encoder Testing", group="Linear Opmode")
public class Encoder extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare a state variable
    int state = -1;

    public Boolean RightBumperPressed = false;
    public Boolean buttonPressed = false;

    public Boolean Uptapped = false;
    public Boolean Downtapped = false;
    public Boolean Lefttapped = false;
    public Boolean Righttapped = false;

    public double wrist = 0;

    private double slowSpeed = 0.65;
    private double fullSpeed = 1;
    private double currentSpeed = 1;
    private boolean canChangeSpeeds = true;


    @Override
    public void runOpMode() throws InterruptedException {
        LiftClass arm = new LiftClass(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up){
                Uptapped = true;
            } else if (gamepad2.dpad_down){
                Downtapped = true;
            } else if (gamepad2.dpad_left){
                Lefttapped = true;
            } else if (gamepad2.dpad_right){
                Righttapped = true;
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
                switch (state){
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
            }else if (Righttapped) {

                switch (state){
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
                        arm.setShoulderPosition(0.585);
                        arm.setElbowPosition(0);
                        state = -1;
                        Righttapped = false;
                        break;
                }
            }else if (Lefttapped) {

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
                        arm.setShoulderPosition(0.5);
                        arm.setElbowPosition(0.125);
                        state = -1;
                        Lefttapped = false;
                        break;
                }
            } else if (gamepad2.options) {
                arm.manualPowerDownPulley();
            } else if (gamepad2.touchpad) {
                arm.manualPowerUpPulley();
            } else if (arm.getCurrentPulleyPosition() != LiftClass.PulleyPosition.HOME){
                arm.zeroPowerPulley();
            }

            if (!gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right){
                buttonPressed = false;
            }

            if (gamepad2.y) {
                arm.setLiftPosition(LiftClass.liftPosition.HOME);
            }  else if (arm.getCurrentLiftPosition() == LiftClass.liftPosition.BASKET) {
                arm.zeroPowerLift();
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double strafe =  gamepad1.left_stick_x;
            double turn     =  gamepad1.right_stick_x;

            if (gamepad1.right_stick_button){
                strafe = 0.25;
            } else if (gamepad1.left_stick_button) {
                strafe = -0.25;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = forward + strafe - turn;
            double rightFrontPower = forward - strafe + turn;
            double leftBackPower   = -forward + strafe + turn;
            double rightBackPower  = -forward - strafe - turn;

            if (gamepad1.y && canChangeSpeeds) {
                canChangeSpeeds = false;
                if (currentSpeed == fullSpeed) {
                    currentSpeed = slowSpeed;
                } else {
                    currentSpeed = fullSpeed;
                }
            } else if (!gamepad1.y) {
                canChangeSpeeds = true;
            }

            // Claw control method
            if (gamepad1.right_bumper || gamepad2.right_bumper && !RightBumperPressed) {
                RightBumperPressed = true;
                if (arm.claw.getPosition() == 1) {
                    arm.claw.setPosition(0);;
                } else {
                    arm.claw.setPosition(1);
                }
            } else if (!gamepad2.right_bumper && !gamepad1.right_bumper) {
                RightBumperPressed = false;
            }

            if (gamepad2.right_trigger > 0.75) {
                arm.setShoulderPosition(0.25);
                arm.setElbowPosition(0);
            } else if (gamepad2.left_trigger > 0.75) {
                arm.setShoulderPosition(0);
                arm.setElbowPosition(0.125);
            } else if (gamepad2.x) {
                arm.setShoulderPosition(0.675);
                arm.setElbowPosition(0.2);
            } else if (gamepad2.left_bumper) {
                arm.setShoulderPosition(1);
                arm.setElbowPosition(0.65);
            }

            if (gamepad2.left_stick_x > .15){
                arm.setWristPosition(wrist);
                wrist += 0.0185;
            } else if (gamepad2.left_stick_x < -0.15) {
                arm.setWristPosition(wrist);
                wrist -= 0.0185;
            }


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * currentSpeed);
            rightFrontDrive.setPower(rightFrontPower * currentSpeed);
            leftBackDrive.setPower(leftBackPower * currentSpeed);
            rightBackDrive.setPower(rightBackPower * currentSpeed);

            telemetry.addData("Left Lift Encoder Position", arm.getLiftPositionL());
            telemetry.addData("Right Lift Encoder Position", arm.getLiftPositionR());

            telemetry.addData("Pulley Right", arm.getPulleyPositionR());
            telemetry.addData("Pulley Left", arm.getPulleyPositionL());

            telemetry.addLine();

            telemetry.addData("Pulley PowerL", arm.PulleyPowerL());
            telemetry.addData("Pulley PowerR", arm.PulleyPowerR());

            telemetry.addLine();

            telemetry.addData("shoulder", arm.ShoulderR.getPosition());
            telemetry.addData("elbow", arm.elbow.getPosition());
            telemetry.addData("wrist", arm.wrist.getPosition());
            telemetry.addData("claw", arm.claw.getPosition());

            telemetry.addData("button pressed", buttonPressed);

            telemetry.update();
        }
    }
}