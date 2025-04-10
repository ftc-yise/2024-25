package org.firstinspires.ftc.teamcode.Swerve;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

/**
 * Constants for swerve drive configuration
 * Contains physical measurements and performance limits
 */
public final class SwerveDriveConstants {
    // Wheel constants - update these when changing wheels
    public static final double WHEEL_CIRCUMFERENCE = 3.5 * Math.PI;  // Circumference in inches
    public static final double TICKS_PER_REVOLUTION = 1;  // Motor encoder ticks per revolution

    // Conversion factor
    private static final double inchesToMeters = 0.0254;  // Conversion from inches to meters

    // Performance limits - may need tuning based on robot capabilities
    public static final double maxSpeedMeters = 2.4638;  // Maximum linear speed in meters/second
    public static final double maxRadiansPerSecond = 3.63 * Math.PI;  // Maximum rotational speed

    // Robot physical dimensions - update these when changing chassis design
    public static final double trackWidth = 11.5;  // Distance between left and right wheels (inches)
    public static final double wheelBase = 12.5;   // Distance between front and back wheels (inches)

    // Convert dimensions to meters for kinematics calculations
    public static final double trackWidthMeters = trackWidth * inchesToMeters;
    public static final double wheelBaseMeters = wheelBase * inchesToMeters;

    // Module positions relative to robot center
    // FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    public static final Translation2d flModuleOffset = new Translation2d(wheelBaseMeters/ 2.0,
            trackWidthMeters / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(wheelBaseMeters / 2.0,
            -trackWidthMeters / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-wheelBaseMeters / 2.0,
            trackWidthMeters / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-wheelBaseMeters / 2.0,
            -trackWidthMeters / 2.0);

    // Create kinematics object to convert chassis motion to module states
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);

    // Driver control scaling - adjust for driver preference
    public static final double rotationMultiplier = 0.5;  // Reduces rotation speed for better control
}