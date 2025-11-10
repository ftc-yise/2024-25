package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.File;

@TeleOp(name="Shooter - Hold Modes (Logged)", group="Ball Bot")
public class Shoota extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // Motors
    private DcMotorEx leftFrontDrive = null;
    private DcMotor leftFrontDriveLegacy = null;

    // --- Mode Control ---
    private enum HoldMode { FLAT, PULSE, PID_VEL }
    private HoldMode holdMode = HoldMode.PULSE;

    // --- Pulse parameters ---
    private double pulseFrequencyHz = 5.0;   // Hz (cycles per second)
    private double pulseDuty = 0.5;          // 50% on-time
    private double pulseOnPower = 0.4;       // power during pulse
    private double flatHoldPower = 0;     // flat mode power

    // --- PID parameters ---
    private double pidTargetTicksPerSec = 2000.0;
    private double kP = 0.0006;
    private double kI = 0.0;
    private double kD = 0.0;
    private double pidOutput = 0.0;
    private double pidIntegral = 0.0;
    private double pidLastError = 0.0;
    private double pidOutputMin = 0.0;
    private double pidOutputMax = 1.0;

    // Debounce helpers
    private boolean prevDpadLeft = false, prevDpadRight = false;
    private boolean prevDpadUp = false, prevDpadDown = false;

    // --- Logging ---
    private PrintWriter logWriter = null;
    private String logFileName;

    @Override
    public void runOpMode() {
        // Try to get DcMotorEx first
        try {
            leftFrontDrive = hardwareMap.get(DcMotorEx.class, "LeftFrontDrive");
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            leftFrontDriveLegacy = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
            leftFrontDriveLegacy.setDirection(DcMotor.Direction.FORWARD);
            leftFrontDriveLegacy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();
        runtime.reset();

        // --- Initialize CSV logging ---
        logFileName = "/sdcard/FIRST/shooter_log_" + System.currentTimeMillis() + ".csv";
        try {
            File logFile = new File(logFileName);
            logWriter = new PrintWriter(new FileWriter(logFile));
            // CSV header
            logWriter.println("time_s,mode,power,velocity_ticks_s,voltage_V,pulseFreq,pulseDuty,pulseOnPower,flatHoldPower,pidTarget,pidOutput");
            logWriter.flush();
            telemetry.addData("Logging to", logFileName);
            telemetry.update();
        } catch (IOException e) {
            telemetry.addData("Log init failed", e.getMessage());
            telemetry.update();
        }

        // --- Main Control Loop ---
        while (opModeIsActive()) {
            double powerToApply = 0.0;

            // Manual overrides
            if (gamepad1.y) powerToApply = 1.0;
            else if (gamepad1.x) powerToApply = 0.75;
            else if (gamepad1.b) powerToApply = 0.65;
            else if (gamepad1.a) powerToApply = 0.6;
            else if (gamepad1.right_bumper) powerToApply = 0.25;
            else if (gamepad1.left_bumper) powerToApply = 0.0;
            else {
                // Mode control
                handleModeSwitching();

                switch (holdMode) {
                    case FLAT:
                        powerToApply = flatHoldPower;
                        break;

                    case PULSE:
                        double t = runtime.seconds();
                        double cyclePos = (t * pulseFrequencyHz) % 1.0;
                        powerToApply = (cyclePos < pulseDuty) ? pulseOnPower : 0.0;
                        break;

                    case PID_VEL:
                        if (leftFrontDrive != null) {
                            double currentVel = leftFrontDrive.getVelocity();
                            double error = pidTargetTicksPerSec - currentVel;
                            pidIntegral += error * getLoopDt();
                            double derivative = (error - pidLastError) / Math.max(1e-6, getLoopDt());
                            pidLastError = error;

                            double output = kP * error + kI * pidIntegral + kD * derivative;
                            pidOutput = clamp(pidOutput + output, pidOutputMin, pidOutputMax);
                            powerToApply = pidOutput;
                        } else {
                            powerToApply = flatHoldPower;
                        }
                        break;
                }
            }

            // Apply power
            setPower(powerToApply);

            // --- Telemetry ---
            double currentVelocity = getVelocityIfAvailable();
            double voltage = battery.getVoltage();
            telemetry.addData("Mode", holdMode);
            telemetry.addData("Power", "%.3f", powerToApply);
            telemetry.addData("Velocity", "%.1f ticks/s", currentVelocity);
            telemetry.addData("Voltage", "%.2f V", voltage);
            telemetry.addData("Pulse Freq", "%.2f Hz", pulseFrequencyHz);
            telemetry.addData("Pulse Duty", "%.2f", pulseDuty);
            telemetry.addData("PID Target", "%.0f", pidTargetTicksPerSec);
            telemetry.update();

            // --- CSV Logging ---
            if (logWriter != null) {
                double tNow = runtime.seconds();
                logWriter.printf("%.3f,%s,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.3f%n",
                        tNow, holdMode, powerToApply, currentVelocity, voltage,
                        pulseFrequencyHz, pulseDuty, pulseOnPower,
                        flatHoldPower, pidTargetTicksPerSec, pidOutput);
                logWriter.flush();
            }
        }

        // --- Close file at end ---
        if (logWriter != null) {
            logWriter.close();
        }
    }

    // --- Helper Functions ---
    private double getLoopDt() { return 0.05; }

    private void handleModeSwitching() {
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadRight && !prevDpadRight) {
            if (holdMode == HoldMode.FLAT) holdMode = HoldMode.PULSE;
            else if (holdMode == HoldMode.PULSE) holdMode = HoldMode.PID_VEL;
            else holdMode = HoldMode.FLAT;
        }

        if (dpadLeft && !prevDpadLeft) {
            if (holdMode == HoldMode.FLAT) holdMode = HoldMode.PID_VEL;
            else if (holdMode == HoldMode.PULSE) holdMode = HoldMode.FLAT;
            else holdMode = HoldMode.PULSE;
        }

        if (holdMode == HoldMode.PULSE) {
            if (dpadUp && !prevDpadUp) pulseDuty = clamp(pulseDuty + 0.05, 0.05, 0.95);
            if (dpadDown && !prevDpadDown) pulseDuty = clamp(pulseDuty - 0.05, 0.05, 0.95);
        }

        if (holdMode == HoldMode.PID_VEL) {
            if (dpadUp && !prevDpadUp) pidTargetTicksPerSec += 100.0;
            if (dpadDown && !prevDpadDown) pidTargetTicksPerSec = Math.max(0.0, pidTargetTicksPerSec - 100.0);
        }

        prevDpadLeft = dpadLeft;
        prevDpadRight = dpadRight;
        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
    }

    private void setPower(double p) {
        p = clamp(p, -1.0, 1.0);
        if (leftFrontDrive != null) leftFrontDrive.setPower(p);
        else if (leftFrontDriveLegacy != null) leftFrontDriveLegacy.setPower(p);
    }

    private double getVelocityIfAvailable() {
        if (leftFrontDrive != null) {
            try {
                return leftFrontDrive.getVelocity();
            } catch (Exception e) { return Double.NaN; }
        }
        return Double.NaN;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
