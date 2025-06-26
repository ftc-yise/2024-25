package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Axon drive", group="Linear OpMode")
public class AxonMechTest  extends LinearOpMode {

        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;

        private CRServo leftFrontAxon = null;
        private CRServo leftBackAxon = null;
        private CRServo rightFrontAxon = null;
        private CRServo rightBackAxon = null;

        private AnalogInput frontLeftAnalog = null;
        private AnalogInput frontRightAnalog = null;
        private AnalogInput backLeftAnalog = null;
        private AnalogInput backRightAnalog = null;


        @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

            leftBackAxon = hardwareMap.get(CRServo.class, "LeftBackAxon");
            leftFrontAxon = hardwareMap.get(CRServo.class, "LeftFrontAxon");
            rightBackAxon = hardwareMap.get(CRServo.class, "RightBackAxon");
            rightFrontAxon = hardwareMap.get(CRServo.class, "RightFrontAxon");

            frontLeftAnalog = hardwareMap.get(AnalogInput.class, "LeftFrontAnalog");
            /* frontRightAnalog = hardwareMap.get(AnalogInput.class, "RightFrontAnalog");
            backLeftAnalog = hardwareMap.get(AnalogInput.class, "LeftBackAnalog");
            backRightAnalog = hardwareMap.get(AnalogInput.class, "RightBackAnalog");
            */
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                while (opModeIsActive()) {
                    double forward = -gamepad1.left_stick_y;
                    double forwardV2 = -gamepad1.left_stick_x;

                    double leftFrontPower = forward + forwardV2;
                    double rightFrontPower = forward + forwardV2;
                    double leftBackPower = forward + forwardV2;
                    double rightBackPower = forward + forwardV2;

                    double threshold = 0.05;  // Joystick deadzone
                    boolean joystickMoved = Math.abs(gamepad1.left_stick_x) > threshold || Math.abs(gamepad1.left_stick_y) > threshold;

                    if (joystickMoved) {
                        // Calculate target angle
                        double targetAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y));

                        // Read the single analog sensor (Front Left)
                        double voltage = frontLeftAnalog.getVoltage();
                        double currentAngle = (voltage / 3.3) * 360.0;

                        double angleDiff = targetAngle - currentAngle;
                        if (angleDiff > 180) angleDiff -= 360;
                        if (angleDiff < -180) angleDiff += 360;

                        double servoPower = targetAngle/360;

                        if (Math.abs(angleDiff) > 10.0) {
                            // Apply steering
                            leftFrontAxon.setPower(servoPower);
                            leftBackAxon.setPower(servoPower);
                            rightFrontAxon.setPower(servoPower);
                            rightBackAxon.setPower(servoPower);
                        } else {
                            // Close enough to desired direction — hold position
                            leftFrontAxon.setPower(0.0);
                            leftBackAxon.setPower(0.0);
                            rightFrontAxon.setPower(0.0);
                            rightBackAxon.setPower(0.0);
                        }

                        telemetry.addData("Target Angle", targetAngle);
                        telemetry.addData("Current Angle", currentAngle);
                        telemetry.addData("Angle Diff", angleDiff);
                        telemetry.addData("Servo Power", servoPower);
                    } else {
                        // No steering input — don't move servos
                        leftFrontAxon.setPower(0.0);
                        leftBackAxon.setPower(0.0);
                        rightFrontAxon.setPower(0.0);
                        rightBackAxon.setPower(0.0);
                    }



                    leftFrontDrive.setPower(leftFrontPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    leftBackDrive.setPower(leftBackPower);
                    rightBackDrive.setPower(rightBackPower);

                    telemetry.addData("Speed", leftFrontAxon.getPower());
                    telemetry.update();
                }

            }
        }
}