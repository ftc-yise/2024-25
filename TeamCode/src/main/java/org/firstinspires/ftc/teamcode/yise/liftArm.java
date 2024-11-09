package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class liftArm {
    // Arm Variables
    public DcMotor armLeft, armRight;
    public enum armPosition {
        DOWN,
        UP
    }
    public armPosition currentArmPosition;
    public double armMotorPower;
    public double leftArmMotorPositionValue;
    public double rightArmMotorPositionValue;

    // Constructor
    public liftArm(HardwareMap hardwareMap) {
        //Initialize arm motors
        armLeft = hardwareMap.get(DcMotor.class, "armleft");
        armRight = hardwareMap.get(DcMotor.class, "armright");

        //Set motor directions
        armRight.setDirection(DcMotor.Direction.FORWARD);
        armLeft.setDirection(DcMotor.Direction.REVERSE);

        //Reset arm motor encoders
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setArmPosition(liftArm.armPosition targetArmPosition) {
        // Make sure arm motors are using encoders
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        switch (targetArmPosition) {
            case DOWN:
                armLeft.setTargetPosition(0);
                armRight.setTargetPosition(0);
                armMotorPower = 1.00;
                break;
            case UP:
                armLeft.setTargetPosition(600);
                armRight.setTargetPosition(600);
                armMotorPower = 1.00;
                break;
        }
        // Run motors to position using defined power level
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(armMotorPower);
        armRight.setPower(armMotorPower);

        leftArmMotorPositionValue = armLeft.getCurrentPosition();
        rightArmMotorPositionValue = armRight.getCurrentPosition();
    }
    public void zeroPower() {
        if (!armLeft.isBusy()) {
            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armLeft.setPower(0.01);
            armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRight.setPower(0.01);
        }
    }

    public void manualPowerUp() {
            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armLeft.setPower(1);
            armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRight.setPower(1);
    }
    public void manualPowerDown() {
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setPower(-1);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setPower(-1);
    }
}
