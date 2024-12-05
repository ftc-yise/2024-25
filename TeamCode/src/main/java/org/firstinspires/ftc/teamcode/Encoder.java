package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.yise.liftArm;

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
    public double wrist = 0;

    private double slowSpeed = 0.65;
    private double fullSpeed = 1;
    private double currentSpeed = 1;
    private boolean canChangeSpeeds = true;


    @Override
    public void runOpMode() throws InterruptedException {
        liftArm arm = new liftArm(hardwareMap);

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

            if (gamepad2.dpad_down) {
                switch (state) {
                    case -1:
                        state = 0; // Initialize the sequence
                        break;
                    case 0:
                        arm.setLiftPosition(liftArm.liftPosition.BASKET);
                        if (arm.getLiftPositionL() >= 200) { // Replace with your own position checking logic
                            state++;
                        }
                        break;
                    case 1:
                        arm.setPulleyPosition(liftArm.PulleyPosition.HOME);





                    arm.setShoulderPosition(0);
                    arm.setElbowPosition(0.125);
                }
            } else if (gamepad2.dpad_up) {
                arm.setPulleyPosition(liftArm.PulleyPosition.BASKET);
            }else if (gamepad2.dpad_right) {
                arm.setPulleyPosition(liftArm.PulleyPosition.SEARCH);
            }else if (gamepad2.dpad_left) {
                arm.setPulleyPosition(liftArm.PulleyPosition.SUBMERSABLE);
            } else if (gamepad2.options) {
                arm.manualPowerDownPulley();
            } else if (gamepad2.touchpad) {
                arm.manualPowerUpPulley();
            } else if (arm.getCurrentPulleyPosition() != liftArm.PulleyPosition.HOME){
                arm.zeroPowerPulley();
            }

            if (gamepad2.y) {
                arm.setLiftPosition(liftArm.liftPosition.HOME);
            }else if (gamepad2.b) {
                arm.setLiftPosition(liftArm.liftPosition.SUBMERSABLE);
            }  else if (arm.getCurrentLiftPosition() == liftArm.liftPosition.BASKET) {
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
            if (gamepad2.right_bumper && !RightBumperPressed) {
                RightBumperPressed = true;
                if (arm.claw.getPosition() == 1) {
                    arm.claw.setPosition(0);;
                } else {
                    arm.claw.setPosition(1);
                }
            } else if (!gamepad2.right_bumper) {
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

            telemetry.update();
        }
    }
}