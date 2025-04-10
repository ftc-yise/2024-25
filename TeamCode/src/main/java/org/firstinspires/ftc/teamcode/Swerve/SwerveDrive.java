package org.firstinspires.ftc.teamcode.Swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Main swerve drive class that controls all four swerve modules
 * Handles driving, kinematics, and IMU integration
 */
public class SwerveDrive extends SubsystemBase {
    // Array of swerve modules (FL, FR, BL, BR)
    private SwerveModule[] modules;

    // Controllers for odometry/path following (currently unused)
    private final PIDFController driveController = new PIDFController(0.01, 0, 0, 0);
    private final PIDFController turnController = new PIDFController(0.018, 0.005, 0.025, 0);

    // IMU for field-relative driving
    private BHI260IMU imu;

    // Time tracking for velocity calculations (unused in current implementation)
    private double period = 0;
    private double lastTimeStamp = 0;

    // Telemetry for debugging
    public Telemetry telemetry;

    /**
     * Constructor for the swerve drive system
     * @param opMode OpMode reference for hardware mapping
     */
    public SwerveDrive(CommandOpMode opMode) {
        // Create configurations for each module
        // Parameters: moduleNumber, drive controller, turn controller, motor name, servo name,
        // encoder name, angle offset, servo direction
        SwerveModuleConfig fl = new SwerveModuleConfig(0, driveController, turnController,
                "driveMotor1", "angleServo1", "angleInput1", 113.9, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig fr = new SwerveModuleConfig(1, driveController, turnController,
                "driveMotor2", "angleServo2", "angleInput2", 33.6, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig bl = new SwerveModuleConfig(2, driveController, turnController,
                "driveMotor3", "angleServo3", "angleInput3", 48.2, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig br = new SwerveModuleConfig(3, driveController, turnController,
                "driveMotor4", "angleServo4", "angleInput4", 125.8, DcMotorSimple.Direction.REVERSE);

        // Create array of modules
        modules = new SwerveModule[] {
                new SwerveModule(fl, opMode),
                new SwerveModule(fr, opMode),
                new SwerveModule(bl, opMode),
                new SwerveModule(br, opMode)
        };

        // Initialize IMU
        imu = opMode.hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        // Set up telemetry with FTC Dashboard integration
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    /**
     * Drive the robot using translation, strafe, and rotation inputs
     * @param translation Forward/backward input (-1.0 to 1.0)
     * @param strafe Left/right input (-1.0 to 1.0)
     * @param rotation Rotation input (-1.0 to 1.0)
     * @param fieldRelative Whether to use field-relative control
     */
    public void drive(double translation, double strafe, double rotation, boolean fieldRelative) {
        // Scale inputs to actual speeds
        double new_translation = translation * SwerveDriveConstants.maxSpeedMeters;
        double new_strafe = strafe * SwerveDriveConstants.maxSpeedMeters;
        double new_rotation = rotation * SwerveDriveConstants.maxRadiansPerSecond * SwerveDriveConstants.rotationMultiplier;

        // Create ChassisSpeeds object (either field or robot relative)
        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(new_translation, new_strafe, new_rotation, getHeading())
                : new ChassisSpeeds(new_translation, new_strafe, new_rotation);

        /* Discretization section - commented out in original code
        // This section would handle converting continuous motion to discrete timesteps
        // May improve tracking accuracy but not currently used
        double currentTimeStamp = (double) System.nanoTime() / 1E9; //to seconds
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        if (period == 0) period = 1/40;

        lastTimeStamp = currentTimeStamp;

        speeds = discretize(speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                period);
        */

        // Convert chassis speeds to individual module states
        SwerveModuleState[] states = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(speeds);

        // Ensure no module tries to go faster than the maximum speed
        SwerveDriveKinematics.normalizeWheelSpeeds(states, SwerveDriveConstants.maxSpeedMeters);

        // Set each module to its calculated state
        for (SwerveModule module : modules) {
            module.setState(states[module.moduleNumber]);
        }
    }

    /**
     * Get the current heading of the robot from the IMU
     * @return Rotation2d representing the robot's heading
     */
    public Rotation2d getHeading() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    /**
     * Reset the IMU's yaw reading
     */
    public void resetYaw() {
        imu.resetYaw();
    }

    /**
     * Discrete-time kinematics for more accurate motion
     * (Imported from WPILib - currently unused)
     */
    public ChassisSpeeds discretize(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double dtSeconds) {

        Pose2d desiredDeltaPose =
                new Pose2d(
                        vxMetersPerSecond * dtSeconds,
                        vyMetersPerSecond * dtSeconds,
                        new Rotation2d(omegaRadiansPerSecond * dtSeconds));

        Twist2d twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
    }

    /**
     * Periodic update method called by command scheduler
     * Currently just updates telemetry with IMU data
     */
    @Override
    public void periodic() {
        // Get and display IMU angles
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Heading", angles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Roll", angles.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Pitch", angles.getPitch(AngleUnit.DEGREES));
    }
}