package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModuleConfig;

import java.util.ArrayList;

@TeleOp(name = "PID Auto Tuner", group = "Test")
@Config
public class PIDServoAutotuner extends CommandOpMode {

    // Tuning parameters (adjustable via dashboard)
    public static double startP = 0.01;
    public static double maxP = 0.05;
    public static double pStep = 0.002;
    public static double dStep = 0.0005;
    public static double testAngle = 90;
    public static double settleTime = 2.0; // seconds to wait for settling
    public static double oscillationThreshold = 5.0; // degrees
    public static int maxTests = 20;

    private ElapsedTime timer = new ElapsedTime();
    private ArrayList<Double> angleHistory = new ArrayList<>();

    // Tuning state machine
    private enum TuningState {
        IDLE,
        TESTING_P,
        TESTING_D,
        COMPLETE,
        MANUAL_TEST
    }

    private TuningState currentState = TuningState.IDLE;
    private double currentP = startP;
    private double currentD = 0;
    private double bestP = 0;
    private double bestD = 0;
    private int testCount = 0;
    private boolean foundGoodP = false;

    // Test results
    private double settleError = 0;
    private double maxOscillation = 0;
    private boolean isOscillating = false;

    PIDFController driveController = new PIDFController(0.01, 0, 0, 0);
    PIDFController turnController = new PIDFController(0.01, 0, 0, 0); // Start with initial values
    SwerveModuleConfig config;
    SwerveModule module;

    public void initialize() {
        config = new SwerveModuleConfig(0, "0FL", driveController, turnController,
                "RightFrontDrive", "RightFrontAxon", "RightFrontAnalog", 0, DcMotorSimple.Direction.FORWARD);
        module = new SwerveModule(config, this);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            run();

            switch (currentState) {
                case IDLE:
                    handleIdleState();
                    break;
                case TESTING_P:
                    handlePTesting();
                    break;
                case TESTING_D:
                    handleDTesting();
                    break;
                case COMPLETE:
                    handleComplete();
                    break;
                case MANUAL_TEST:
                    handleManualTest();
                    break;
            }

            updateTelemetry();
        }
    }

    private void handleIdleState() {
        if (gamepad1.a) {
            startAutoTuning();
        }
        if (gamepad1.x) {
            startManualTest();
        }
    }

    private void startAutoTuning() {
        currentState = TuningState.TESTING_P;
        currentP = startP;
        currentD = 0;
        testCount = 0;
        foundGoodP = false;
        updatePID();
        startTest();
    }

    private void startManualTest() {
        currentState = TuningState.MANUAL_TEST;
        updatePID();
        startTest();
    }

    private void handlePTesting() {
        if (timer.seconds() > settleTime) {
            analyzeResults();

            telemetry.addData("DEBUG P", "Error: %.2f, Osc: %.2f, IsOsc: %b", settleError, maxOscillation, isOscillating);
            telemetry.update();

            if (isOscillating || settleError > 10) {
                // P too high, we've found our limit
                if (testCount > 0) {
                    bestP = currentP - pStep; // Use previous value
                    foundGoodP = true;
                    currentP = bestP;
                    currentD = 0; // Reset D for D testing
                    currentState = TuningState.TESTING_D;
                    testCount = 0; // Reset test count for D testing
                    updatePID();
                    startTest();
                } else {
                    // Even minimum P oscillates, reduce step size
                    pStep = Math.max(0.0001, pStep * 0.5);
                    currentP = Math.max(0.0005, currentP - pStep);
                    updatePID();
                    startTest();
                }
            } else if (settleError > 3) {
                // P too low, increase
                currentP += pStep;
                testCount++;
                if (currentP > maxP || testCount > maxTests) {
                    bestP = currentP;
                    foundGoodP = true;
                    currentD = 0; // Reset D for D testing
                    currentState = TuningState.TESTING_D;
                    testCount = 0; // Reset test count for D testing
                    updatePID();
                    startTest();
                } else {
                    updatePID();
                    startTest();
                }
            } else {
                // Good P value found
                bestP = currentP;
                foundGoodP = true;
                currentD = 0; // Reset D for D testing
                currentState = TuningState.TESTING_D;
                testCount = 0; // Reset test count for D testing
                updatePID();
                startTest();
            }
        } else {
            // Still testing, collect data
            collectAngleData();
        }
    }

    private void handleDTesting() {
        if (timer.seconds() > settleTime) {
            analyzeResults();

            telemetry.addData("DEBUG D", "Error: %.2f, Osc: %.2f, IsOsc: %b", settleError, maxOscillation, isOscillating);
            telemetry.update();

            if (isOscillating) {
                // D too high, use previous value
                if (currentD > 0) {
                    bestD = Math.max(0, currentD - dStep);
                } else {
                    bestD = 0;
                }
                currentState = TuningState.COMPLETE;
            } else if (maxOscillation > 2 && settleError > 1) {
                // Still some overshoot, try more D
                currentD += dStep;
                testCount++;
                if (currentD > 0.01 || testCount > maxTests) {
                    bestD = Math.max(0, currentD - dStep);
                    currentState = TuningState.COMPLETE;
                } else {
                    updatePID();
                    startTest();
                }
            } else {
                // Good D value
                bestD = currentD;
                currentState = TuningState.COMPLETE;
            }
        } else {
            collectAngleData();
        }
    }

    private void handleComplete() {
        // Set final tuned values
        currentP = bestP;
        currentD = bestD;
        updatePID();

        if (gamepad1.b) {
            currentState = TuningState.IDLE;
        }
        if (gamepad1.y) {
            // Test final values
            startTest();
            currentState = TuningState.MANUAL_TEST;
        }
    }

    private void handleManualTest() {
        boolean updated = false;

        if (gamepad1.dpad_up) {
            currentP += 0.001;
            updated = true;
        }
        if (gamepad1.dpad_down) {
            currentP = Math.max(0, currentP - 0.001);
            updated = true;
        }
        if (gamepad1.dpad_right) {
            currentD += 0.0002;
            updated = true;
        }
        if (gamepad1.dpad_left) {
            currentD = Math.max(0, currentD - 0.0002);
            updated = true;
        }

        if (updated) {
            updatePID();
        }

        if (gamepad1.right_bumper) {
            startTest();
        }

        if (gamepad1.b) {
            currentState = TuningState.IDLE;
        }

        if (timer.seconds() > settleTime) {
            analyzeResults();
        } else {
            collectAngleData();
        }
    }

    private void startTest() {
        angleHistory.clear();
        timer.reset();
        module.setState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(testAngle))));
    }

    private void updatePID() {
        // Actually update the PID controller with new values
        turnController.setPIDF(currentP, 0, currentD, 0);
        telemetry.addData("PID Updated", "P: %.4f, D: %.4f", currentP, currentD);
    }

    private void collectAngleData() {
        double currentAngle = module.getWheelAngleRad();
        angleHistory.add(Math.toDegrees(currentAngle));

        // Keep only recent data (last 1 second at ~50Hz)
        if (angleHistory.size() > 50) {
            angleHistory.remove(0);
        }
    }

    private void analyzeResults() {
        if (angleHistory.size() < 10) return;

        double targetAngle = testAngle;
        double currentAngle = Math.toDegrees(module.getWheelAngleRad());

        // Calculate settle error
        settleError = Math.abs(currentAngle - targetAngle);

        // Calculate oscillation (standard deviation of recent angles)
        double sum = 0;
        for (double angle : angleHistory) {
            sum += angle;
        }
        double mean = sum / angleHistory.size();

        double variance = 0;
        for (double angle : angleHistory) {
            variance += Math.pow(angle - mean, 2);
        }
        maxOscillation = Math.sqrt(variance / angleHistory.size());

        // Check if oscillating - more sensitive detection
        isOscillating = maxOscillation > oscillationThreshold;

        // Additional oscillation check: look for rapid direction changes
        if (!isOscillating && angleHistory.size() >= 20) {
            int directionChanges = 0;
            for (int i = 2; i < angleHistory.size(); i++) {
                double prev2 = angleHistory.get(i-2);
                double prev1 = angleHistory.get(i-1);
                double curr = angleHistory.get(i);

                if ((prev1 - prev2) * (curr - prev1) < 0) {
                    directionChanges++;
                }
            }
            // If more than 30% of samples show direction changes, it's oscillating
            if ((double)directionChanges / (angleHistory.size() - 2) > 0.3) {
                isOscillating = true;
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("=== AUTO TUNER ===", "");
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Test Count", testCount);
        telemetry.addData("Current P", "%.4f", currentP);
        telemetry.addData("Current D", "%.4f", currentD);
        telemetry.addData("Best P", "%.4f", bestP);
        telemetry.addData("Best D", "%.4f", bestD);
        telemetry.addData("", "");

        telemetry.addData("=== TEST RESULTS ===", "");
        telemetry.addData("Current Angle", "%.1f°", Math.toDegrees(module.getWheelAngleRad()));
        telemetry.addData("Target Angle", "%.1f°", testAngle);
        telemetry.addData("Settle Error", "%.2f°", settleError);
        telemetry.addData("Oscillation", "%.2f°", maxOscillation);
        telemetry.addData("Is Oscillating", isOscillating);
        telemetry.addData("Test Time", "%.1f s", timer.seconds());
        telemetry.addData("Data Points", angleHistory.size());
        telemetry.addData("", "");

        telemetry.addData("=== CONTROLS ===", "");
        if (currentState == TuningState.IDLE) {
            telemetry.addData("A", "Start Auto Tuning");
            telemetry.addData("X", "Manual Test Mode");
        } else if (currentState == TuningState.MANUAL_TEST) {
            telemetry.addData("D-Pad Up/Down", "Adjust P (±0.001)");
            telemetry.addData("D-Pad Left/Right", "Adjust D (±0.0002)");
            telemetry.addData("Right Bumper", "Test Current Values");
            telemetry.addData("B", "Back to Idle");
        } else if (currentState == TuningState.COMPLETE) {
            telemetry.addData("TUNING COMPLETE!", "");
            telemetry.addData("Final P", "%.4f", bestP);
            telemetry.addData("Final D", "%.4f", bestD);
            telemetry.addData("Y", "Test Final Values");
            telemetry.addData("B", "Back to Idle");
        } else {
            telemetry.addData("Auto tuning in progress...", "");
            telemetry.addData("Please wait", "");
        }

        telemetry.update();
    }
}