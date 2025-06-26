package org.firstinspires.ftc.teamcode.Swerve;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

 /**
 * Configuration class for each swerve module
 * This class stores all the hardware mappings and tuning parameters needed
 * for a single swerve module
 */

public class SwerveModuleConfig {

    public final String driveMotorName, angleServoName, absoluteEncoderName;
    public final double offset;
    public final int moduleNumber;
     public final String moduleString;

     public PIDFController drivePIDFController, anglePIDFController;
    public DcMotorSimple.Direction angleReverse;

    public SwerveModuleConfig(int modNumber, String moduleString, PIDFController drivePIDFController, PIDFController anglePIDFController,
                              String driveMotorName, String angleServoName, String absoluteEncoderName, double offset, DcMotorSimple.Direction angleReverse) {
        this.driveMotorName = driveMotorName;
        this.angleServoName = angleServoName;
        this.absoluteEncoderName = absoluteEncoderName;
        this.drivePIDFController = drivePIDFController;
        this.anglePIDFController = anglePIDFController;
        this.angleReverse = angleReverse;
        this.offset = offset;
        this.moduleNumber = modNumber;
        this.moduleString = moduleString;
    }
}
