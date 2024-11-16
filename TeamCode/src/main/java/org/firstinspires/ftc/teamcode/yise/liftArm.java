package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class liftArm {
    // Arm Variables
    public DcMotor liftLeft, liftRight, pulleyLeft, pulleyRight;
    public enum armPosition {
        DOWN,
        UP
    }

    public enum PulleyPosition {
        DOWN,
        UP
    }

    public armPosition currentArmPosition;
    public double armMotorPower;
    public double pulleyMotorPower;
    public double leftArmMotorPositionValue;
    public double rightArmMotorPositionValue;

    // Constructor
    public liftArm(HardwareMap hardwareMap) {
        //Initialize arm motors
        liftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        liftRight = hardwareMap.get(DcMotor.class, "liftright");

        pulleyLeft = hardwareMap.get(DcMotor.class, "pulleyLeft");
        pulleyRight = hardwareMap.get(DcMotor.class, "pulleyRight");

        //Set motor directions
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        pulleyLeft.setDirection(DcMotor.Direction.REVERSE);

        //Reset arm motor encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pulleyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleyRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setLiftPosition(liftArm.armPosition targetArmPosition) {
        // Make sure arm motors are using encoders
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        switch (targetArmPosition) {
            case DOWN:
                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                armMotorPower = 1.00;
                break;
            case UP:
                liftLeft.setTargetPosition(600);
                liftRight.setTargetPosition(600);
                armMotorPower = 1.00;
                break;
        }
        // Run motors to position using defined power level
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(armMotorPower);
        liftRight.setPower(armMotorPower);

        leftArmMotorPositionValue = liftLeft.getCurrentPosition();
        rightArmMotorPositionValue = liftRight.getCurrentPosition();
    }

    public void setPulleyPosition(liftArm.PulleyPosition targetPulleyPosition) {
        // Make sure arm motors are using encoders
        pulleyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulleyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        switch (targetPulleyPosition) {
            case DOWN:
                pulleyLeft.setTargetPosition(0);
                pulleyRight.setTargetPosition(0);
                pulleyMotorPower = 1.00;
                break;
            case UP:
                pulleyLeft.setTargetPosition(1500);
                pulleyRight.setTargetPosition(1500);
                pulleyMotorPower = 1.00;
                break;
        }
        // Run motors to position using defined power level
        pulleyLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyLeft.setPower(pulleyMotorPower);
        pulleyRight.setPower(pulleyMotorPower);
    }

    public void zeroPowerLift() {
        if (!liftLeft.isBusy()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(0.01);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.01);
        }
    }

    public void zeroPowerPulley() {
        if (!liftLeft.isBusy()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(0.01);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.01);
        }
    }

    public void manualPowerUpLift() {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(1);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(1);
    }
    public void manualPowerDownLift() {
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(-1);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setPower(-1);
    }

    public void manualPowerUpPulley() {
        pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyLeft.setPower(1);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(1);
    }
    public void manualPowerDownPulley() {
        pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyLeft.setPower(-1);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(-1);
    }


    public double getPulleyPositionL() {
        return pulleyLeft.getCurrentPosition();
    }
    public double getPulleyPositionR() {
        return pulleyRight.getCurrentPosition();
    }

    public double getLiftPositionL() {
        return liftLeft.getCurrentPosition();
    }
    public double getLiftPositionR() {
        return liftRight.getCurrentPosition();
    }
}
