package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp(name="BB Main Drive (Field-Oriented Toggle)", group="Ball Bot")
public class BallBotMainDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private ColorSensor color = null;

    // Intake (untouched)
    private DcMotor intake = null;

    // Speed scaling
    private double slowSpeed = 0.4;
    private double fullSpeed = 1.0;
    private double currentSpeed = 1.0;
    private boolean canChangeSpeeds = true;

    // Acceleration smoothing
    private double rampRate = 0.08;
    private double lastLF = 0, lastLB = 0, lastRF = 0, lastRB = 0;

    // Orientation toggle
    private boolean fieldOriented = false;
    private boolean canToggleFOD = true;

    // Replace this with your actual OTOS localizer reference
    // For example: private MyOtosLocalizer otos;
    private double getHeadingRadians() {
        // Example placeholder — replace with your real OTOS heading getter
        // Must return heading in radians, where 0 = field-forward, positive CCW
        return Math.toRadians(0);
    }

    @Override
    public void runOpMode() {
        // --- Hardware map ---
        color = hardwareMap.get(ColorSensor.class, "color");


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        setBrakeMode(true);

        // Initialize OTOS if needed
        // otos = new MyOtosLocalizer(hardwareMap);
        // otos.reset();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double deadband = 0.05;

            // --- Speed toggle ---
            if (gamepad1.y && canChangeSpeeds) {
                canChangeSpeeds = false;
                currentSpeed = (currentSpeed == fullSpeed) ? slowSpeed : fullSpeed;
            } else if (!gamepad1.y) {
                canChangeSpeeds = true;
            }

            // --- Field/robot orientation toggle ---
            if (gamepad1.a && canToggleFOD) {
                canToggleFOD = false;
                fieldOriented = !fieldOriented;
            } else if (!gamepad1.a) {
                canToggleFOD = true;
            }

            // --- Intake ---
            if (gamepad1.right_trigger > 0.75 || gamepad1.left_trigger > 0.75) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // --- Joystick inputs ---
            double x = applyDeadband(gamepad1.left_stick_x, deadband);
            double y = -applyDeadband(gamepad1.left_stick_y, deadband); // invert for forward
            double turn = -applyDeadband(gamepad1.right_stick_x, deadband);

            // === FIELD ORIENTED TRANSFORM ===
            if (fieldOriented) {
                double heading = getHeadingRadians();  // from OTOS
                double cosA = Math.cos(heading);
                double sinA = Math.sin(heading);

                double fieldX = x * cosA - y * sinA;
                double fieldY = x * sinA + y * cosA;

                x = fieldX;
                y = fieldY;
            }

            // === Mecanum drive power calculation ===
            double lf = y + x + turn;
            double rf = y - x - turn;
            double lb = y - x + turn;
            double rb = y + x - turn;

            // Normalize
            double max = Math.max(1.0, Math.abs(lf));
            max = Math.max(max, Math.abs(rf));
            max = Math.max(max, Math.abs(lb));
            max = Math.max(max, Math.abs(rb));

            lf /= max; rf /= max; lb /= max; rb /= max;

            // Scale
            lf *= currentSpeed; rf *= currentSpeed; lb *= currentSpeed; rb *= currentSpeed;

            // Ramping
            lf = ramp(lastLF, lf, rampRate);
            rf = ramp(lastRF, rf, rampRate);
            lb = ramp(lastLB, lb, rampRate);
            rb = ramp(lastRB, rb, rampRate);

            lastLF = lf; lastRF = rf; lastLB = lb; lastRB = rb;

            // Smart braking
            if (Math.abs(x) < 0.05 && Math.abs(y) < 0.05 && Math.abs(turn) < 0.05)
                setBrakeMode(true);
            else
                setBrakeMode(false);

            // Set motor power
            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);

            // --- Telemetry ---
            telemetry.addData("Speed Mode", currentSpeed == fullSpeed ? "FULL" : "SLOW");
            telemetry.addData("Orientation", fieldOriented ? "FIELD" : "ROBOT");
            telemetry.addData("Heading (deg)", Math.toDegrees(getHeadingRadians()));
            telemetry.addData("LF/RF/LB/RB", "%.2f %.2f %.2f %.2f", lf, rf, lb, rb);

            telemetry.addLine();
            telemetry.addData("color Red Chanel", color.red());
            telemetry.addData("color Blue Chanel", color.blue());
            telemetry.addData("color Green Chanel", color.green());
            telemetry.addData("color ARBG", color.argb());
            telemetry.addData("color Alpha", color.alpha());



            telemetry.update();
        }
    }

    // --- Utility Functions ---

    private double applyDeadband(double val, double threshold) {
        return (Math.abs(val) > threshold) ? val : 0.0;
    }

    private double ramp(double last, double target, double rate) {
        if (Math.abs(target - last) > rate) {
            return last + Math.signum(target - last) * rate;
        } else {
            return target;
        }
    }

    private void setBrakeMode(boolean enabled) {
        DcMotor.ZeroPowerBehavior mode = enabled ?
                DcMotor.ZeroPowerBehavior.BRAKE :
                DcMotor.ZeroPowerBehavior.FLOAT;

        leftFrontDrive.setZeroPowerBehavior(mode);
        rightFrontDrive.setZeroPowerBehavior(mode);
        leftBackDrive.setZeroPowerBehavior(mode);
        rightBackDrive.setZeroPowerBehavior(mode);
    }
}
