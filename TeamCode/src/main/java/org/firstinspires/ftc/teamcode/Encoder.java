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

    private double slowSpeed = 0.4;
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

            if (gamepad1.dpad_down) {
               arm.setLiftPosition(liftArm.liftPosition.DOWN);
            } else if (gamepad1.dpad_up) {
                arm.setLiftPosition(liftArm.liftPosition.UP);
            } else if (gamepad1.left_bumper) {
                arm.manualPowerDownLift();
            }else if (gamepad1.right_bumper) {
                arm.manualPowerUpLift();
            } else {
                arm.zeroPowerLift();
            }

            if (gamepad1.right_trigger > 0.75) {
               arm.manualPowerUpPulley();
            } else if (gamepad1.left_trigger > 0.75) {
                arm.manualPowerDownPulley();
            }else if (gamepad1.dpad_left) {
                arm.setPulleyPosition(liftArm.PulleyPosition.IN);
            } else if (gamepad1.dpad_right) {
                arm.setPulleyPosition(liftArm.PulleyPosition.OUT);}
            else {
                arm.zeroPowerPulley();
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double strafe =  gamepad1.left_stick_x;
            double turn     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = forward + strafe - turn;
            double rightFrontPower = forward - strafe + turn;
            double leftBackPower   = forward + strafe + turn;
            double rightBackPower  = forward - strafe - turn;

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

// Servo control Step method
            if (gamepad1.a) {
                switch (state) {
                    case -1:
                        state = 0; // Initialize the sequence
                        break;
                    case 0:
                        arm.manualSetElbowPosition(0);
                        if (arm.elbow.getPosition() == 0) { // Replace with your own position checking logic
                            state++;
                        }
                        break;
                    case 1:
                        arm.manualSetShoulderPosition(1);
                        if (arm.shoulderR.getPosition() == 1) { // Replace with your own position checking logic
                            state++;
                        }
                        break;
                    case 2:
                        arm.manualSetWristPosition(0);
                        if (arm.wrist.getPosition() == 0) { // Replace with your own position checking logic
                            state++;
                        }
                        break;
                    case 3:
                        arm.manualSetClawPosition(0);
                        if (arm.claw.getPosition() == 0) { // Replace with your own position checking logic
                            state = -1; // Reset state
                        }
                        break;
                }
            } else if (gamepad1.b) {
                switch (state) {
                    case -1:
                        state = 3; // Initialize the reverse sequence
                        break;
                    case 3:
                        arm.manualSetClawPosition(1);
                        if (arm.claw.getPosition() == 1) { // Replace with your own position checking logic
                            state--;
                        }
                        break;
                    case 2:
                        arm.manualSetWristPosition(1);
                        if (arm.wrist.getPosition() == 1) { // Replace with your own position checking logic
                            state--;
                        }
                        break;
                    case 1:
                        arm.manualSetShoulderPosition(0);
                        if (arm.shoulderR.getPosition() == 0) { // Replace with your own position checking logic
                            state--;
                        }
                        break;
                    case 0:
                        arm.manualSetElbowPosition(1);
                        if (arm.elbow.getPosition() == 1) { // Replace with your own position checking logic
                            state = -1; // Reset state
                        }
                        break;
                }
            }

            if (gamepad1.x){
                arm.manualSetShoulderPosition(1);
            } else if (gamepad1.y) {
                arm.manualSetShoulderPosition(0);
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

            telemetry.addData("shoulder", arm.shoulderR.getPosition());
            telemetry.addData("elbow", arm.elbow.getPosition());
            telemetry.addData("wrist", arm.wrist.getPosition());
            telemetry.addData("claw", arm.claw.getPosition());
            
            telemetry.update();
        }
    }
}