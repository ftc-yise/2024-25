package org.firstinspires.ftc.teamcode.archived23_24SeaonCenterStage.yiseArchived;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RoadRunnerDriving {

    //Drive class
    SampleMecanumDrive drive;

    // Used to track slow-mode versus normal mode
    public Speeds currentSpeed;
    double speedMultiplier;
    public enum Speeds {
        SLOW,
        NORMAL
    }


    //Declare the constructor for the class
    public RoadRunnerDriving(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        drive.setPoseEstimate(PoseStorage.currentPose);

        // set default value for speed
        currentSpeed = Speeds.NORMAL;
        speedMultiplier = 1;
    }

    double x = 0;
    double y = 0;
    double heading = 0;

    //Drives the robot based on gamepad input
    public void updateMotorsFromStick(Gamepad gamepad) {

        //Set drive power based on gamepad inputs multiplied by the speed variable
        if (!gamepad.dpad_down && !gamepad.dpad_up && !gamepad.dpad_left && !gamepad.dpad_right) {
            x = -gamepad.left_stick_y * speedMultiplier;
            y = -gamepad.left_stick_x * speedMultiplier;
            heading = -gamepad.right_stick_x * speedMultiplier;
            drive.setWeightedDrivePower(new Pose2d(x, y, heading));
        }

    }

    public void updateFromDpad(double x, double y, double heading) {
        drive.setWeightedDrivePower(new Pose2d(x, y, heading));
    }


    //Toggles fast and slow speeds
    public void toggleSlowMode(Speeds targetSpeed) {

        // Set the speedMultiplier in case of SLOW mode
        if (currentSpeed == Speeds.SLOW) {
            currentSpeed = Speeds.NORMAL;
            speedMultiplier = 1;
        } else {
            currentSpeed = Speeds.SLOW;
            speedMultiplier = 0.5;
        }
    }

    //Updates the roadrunner coords; must be called every tick
    public void update() {
        drive.update();
    }

    //Accessor to get the robot position
    public Pose2d getPosition() {
        return drive.getPoseEstimate();
    }

    //Calibrate position using the middle april tag

}

