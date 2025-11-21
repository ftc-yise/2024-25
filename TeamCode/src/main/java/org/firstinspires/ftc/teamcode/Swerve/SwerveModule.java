package org.firstinspires.ftc.teamcode.Swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SwerveTrajectoryFollower;

/**
 * Class representing a single swerve module (drive motor + angle servo)
 * Handles control of both the wheel angle and drive speed
 */
public class SwerveModule extends SubsystemBase {

    // Hardware components
    public CRServo angleServo;            // Continuous rotation servo that controls wheel orientation
    public DcMotorEx driveMotor;          // Motor that powers wheel rotation
    public AnalogInput servoPotentiometer; // Analog sensor to measure the wheel angle
    public double angleOffset;            // Calibration offset for the wheel angle
    public double angleSetPoint;          // Target angle for the wheel

    // Controllers
    public CustomPIDFController angleController;  // PID controller for wheel orientation
    public PIDFController driveController;        // PID controller for drive motor (unused in current implementation)

    public int moduleNumber;// Module identifier (0-3)
    public String moduleString;
    public Telemetry telemetry;           // Telemetry for debugging

    // State variables for telemetry and debugging
    public double setpoint = 0;           // Current angle setpoint
    public double anglePID = 0;           // Current angle PID output
    public double wheelDegs = 0;          // Current wheel orientation in degrees
    public double drivePower = 0;         // Current drive motor power
    public double driveSpeedMetersPerSecond = 0; // Current drive speed in m/s

    private final boolean showTelemetry = true;  // Enable/disable telemetry output

    /**
     * Constructor for a swerve module
     * @param config Configuration parameters for this module
     * @param opMode OpMode reference for hardware mapping
     */
    public SwerveModule(SwerveModuleConfig config, SwerveTrajectoryFollower opMode) {
        // Initialize drive motor
        driveMotor = opMode.hardwareMap.get(DcMotorEx.class, config.driveMotorName);

        // Initialize angle servo
        angleServo = opMode.hardwareMap.get(CRServo.class, config.angleServoName);
        angleServo.setDirection(config.angleReverse);

        // Initialize absolute encoder for wheel orientation
        servoPotentiometer = opMode.hardwareMap.get(AnalogInput.class, config.absoluteEncoderName);

        // Set calibration offset for wheel angle
        angleOffset = config.offset;

        // Initialize angle controller with PID values
        // Note: Using custom PID controller instead of the one from config
        angleController = new CustomPIDFController(config.anglePIDFController.getP(), 0, 0, 0);
        angleController.enableContinuousInput(-180, 180);  // Enable wrapping around 180 degrees
        angleController.setTolerance(3);  // Set tolerance to 3 degrees

        // Store drive controller from config
        driveController = config.drivePIDFController;

        // Store module number for identification
        moduleNumber = config.moduleNumber;
        moduleString = config.moduleString;

        // Set up telemetry with FTC Dashboard integration
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    /**
     * Set the desired state (angle and speed) of the module
     * @param state Desired swerve module state
     */
    public void setState(SwerveModuleState state) {
        // Optimize the state to minimize wheel rotation
        SwerveModuleState newState = SwerveModuleState.optimize(state, new Rotation2d(getWheelAngleRad()));

        // Set speed and angle
        setSpeed(newState);
        setAngle(newState);
    }

    /**
     * Set the drive speed of the module
     * @param state Desired swerve module state
     */
    public void setSpeed(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        driveSpeedMetersPerSecond = speed;

        // Convert speed to normalized power (-1.0 to 1.0)
        double power = speed / SwerveDriveConstants.maxSpeedMeters;
        if (power > 1) power = 1;
        if (power < -1) power = -1;

        // Store and apply motor power
        drivePower = power;
        driveMotor.setPower(power);
    }

    /**
     * Set the angle of the module
     * @param state Desired swerve module state
     */
    public void setAngle(SwerveModuleState state) {
        angleSetPoint = state.angle.getDegrees();

        // Update controller setpoint
        setpoint = angleSetPoint;
        angleController.setSetPoint(setpoint);

        // Get current wheel angle
        wheelDegs = getWheelAngleDeg();

        // Calculate PID output for servo
        double pidout = angleController.calculate(wheelDegs);

        // Clamp PID output to valid range
        if (pidout > 1) pidout = 1;
        if (pidout < -1) pidout = -1;

        // Store and apply servo power
        anglePID = pidout;
        angleServo.setPower(pidout);
    }

    /**
     * Get the current state of the module
     * @return Current swerve module state
     */
    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeed();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(angleRadians * 180 / Math.PI));
    }

    /**
     * Get current wheel angle in radians
     * @return Wheel angle in radians
     */
    public double getWheelAngleRad() {
        return getWheelAngleDeg() * Math.PI / 180;
    }

    /**
     * Get current wheel angle in degrees, adjusted by calibration offset
     * @return Wheel angle in degrees
     */
    double getWheelAngleDeg() {
        double volts = servoPotentiometer.getVoltage();
        double potAngle = volts * 360 / 3.3;  // Convert voltage to degrees (3.3V = 360 degrees)
        return Math.IEEEremainder((potAngle + angleOffset), 360);  // Apply offset and wrap to -180 to 180
    }

    /**
     * Get current wheel speed in meters per second
     * @return Wheel speed in m/s
     */
    public double getWheelSpeed() {
        double motorTicksPerSecond = driveMotor.getVelocity();
        double wheelRevolutionsPerSecond = motorTicksPerSecond / SwerveDriveConstants.TICKS_PER_REVOLUTION;
        return wheelRevolutionsPerSecond * SwerveDriveConstants.WHEEL_CIRCUMFERENCE;
    }

    /**
     * Periodic update method called by command scheduler
     * Updates telemetry and stops servo if at setpoint
     */
    @Override
    public void periodic() {
        if (showTelemetry) {
            // Display current module state
            telemetry.addData("CurrentDegrees" + moduleNumber, getWheelAngleDeg());
            telemetry.addData("SetPoint" + moduleNumber, setpoint);
            telemetry.addData("PIDOut" + moduleNumber, anglePID);
            telemetry.addData("DrivePower" + moduleNumber, drivePower);
            telemetry.addData("DriveSpeed" + moduleNumber, driveSpeedMetersPerSecond);
        }

        // Stop the servo if we've reached the setpoint
        if (angleController.atSetPoint()) {
            angleServo.setPower(0);
        }
    }
}