package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModuleConfig;

@TeleOp(name = "SingleSwerveModuleTestBR", group = "Test")
@Config
public class SwerveModuleTestBR extends CommandOpMode {


    private final PIDFController driveController = new PIDFController(0.01, 0, 0, 0);
    private final PIDFController turnController = new PIDFController(0.0048, 0.03, 0.013, 0.001);
    private SwerveModuleConfig config = new SwerveModuleConfig(0, "0FL", driveController, turnController,
            "RightBackDrive", "RightBackAxon", "RightBackAnalog", 2.5, DcMotorSimple.Direction.FORWARD);

    public double targetAngle = 0;

    SwerveModule module;


    public void initialize() {
        module = new SwerveModule(config, this);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            run();
            if (gamepad1.right_bumper) {
                if (gamepad1.a) targetAngle = 0;
                if (gamepad1.b) targetAngle = 45;
                if (gamepad1.x) targetAngle = 90;
                if (gamepad1.y) targetAngle = 135;
                module.setState(new SwerveModuleState(0, new Rotation2d((targetAngle / 180) * Math.PI)));
            } else {
                targetAngle=0;
            }
            // Add these debug lines:
            telemetry.addData("Raw Voltage", module.servoPotentiometer.getVoltage());
            telemetry.addData("Current Angle", module.getWheelAngleRad());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("PID Output", module.anglePID);
            telemetry.addData("At Setpoint", module.angleController.atSetPoint());
            telemetry.update();
        }
    }

}