package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;

@TeleOp(name = "FullSwerveTest", group = "Test")
@Config
public class FullSwerveDriveTest extends CommandOpMode {

    SwerveDrive swerveDrive;
    public Telemetry m_telemetry;

    public void initialize() {
        swerveDrive = new SwerveDrive(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        m_telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            run();
            if (gamepad1.right_bumper) {
                double y = 0.5 * gamepad1.left_stick_y;
                double x = 0.5 * gamepad1.left_stick_x;
                double rx = 0.5 * gamepad1.right_stick_x;
                swerveDrive.drive(y, -x, rx, false);
            }   else {
                swerveDrive.drive(0, 0, 0, false);

            }
            if (gamepad1.a) {
                swerveDrive.resetYaw();
            }
            m_telemetry.update();
        }
    }
}