package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class liftArm {
    // Arm Variables
    public DcMotor liftLeft, liftRight, pulleyLeft, pulleyRight;
    public Servo wrist, claw, shoulderL, shoulderR, elbow;
    public enum liftPosition {
        DOWN,
        UP
    }

    public enum PulleyPosition {
        IN,
        MIDDLE, OUT
    }
    public enum armPosition {
        UP,
        DOWN
    }
    public enum clawPosition {
        OPEN,
        CLOSED
    }
    public enum wristPosition{
       ONE,
       TWO,
       THREE
    }

    public liftPosition currentLiftPosition;
    public double liftMotorPower;
    public double pulleyMotorPower;
    public double leftLiftMotorPositionValue;
    public double rightLiftMotorPositionValue;

    // Constructor
    public liftArm(HardwareMap hardwareMap) {
        //Initialize arm motors
        liftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        liftRight = hardwareMap.get(DcMotor.class, "liftright");

        pulleyLeft = hardwareMap.get(DcMotor.class, "pulleyLeft");
        pulleyRight = hardwareMap.get(DcMotor.class, "pulleyRight");

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        shoulderR = hardwareMap.get(Servo.class, "shoulderR");
        elbow = hardwareMap.get(Servo.class, "elbow");

        //Set motor directions
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        pulleyLeft.setDirection(DcMotor.Direction.REVERSE);
        shoulderL.setDirection(Servo.Direction.REVERSE);

        //Reset arm motor encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pulleyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleyRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        manualSetShoulderPosition(1);
        manualSetElbowPosition(0);
        manualSetWristPosition(0);
        manualSetClawPosition(0);
    }

    public void setLiftPosition(liftPosition targetLiftPosition) {
            switch (targetLiftPosition) {
            case DOWN:
                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                liftMotorPower = 1;
                break;
            case UP:
                liftLeft.setTargetPosition(410);
                liftRight.setTargetPosition(410);
                liftMotorPower = 1;
                break;
        }
        // Run motors to position using defined power level
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(liftMotorPower);
        liftRight.setPower(liftMotorPower);
    }

    public void setPulleyPosition(liftArm.PulleyPosition targetPulleyPosition) {
        switch (targetPulleyPosition) {
            case IN:
                pulleyLeft.setTargetPosition(0);
                pulleyRight.setTargetPosition(0);
                break;
            case OUT:
                pulleyLeft.setTargetPosition(3600);
                pulleyRight.setTargetPosition(3600);
                break;
        }
        // Run motors to position using defined power level
        pulleyLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyLeft.setPower(1);
        pulleyRight.setPower(1);
    }

    public void zeroPowerLift() {
        if (!liftLeft.isBusy() && !liftRight.isBusy()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(0.05);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.05);
        }
    }

    public void zeroPowerPulley() {
        if (!pulleyLeft.isBusy() && !pulleyRight.isBusy()) {
            pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyLeft.setPower(0.01);
            pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyRight.setPower(0.01);
        }
    }

    public void manualPowerUpLift() {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(0.35);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.35);
    }
    public void manualPowerDownLift() {
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(-0.35);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setPower(-0.35);
    }

    public void manualPowerUpPulley() {
        pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyLeft.setPower(1);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(1);
    }
    public void manualPowerDownPulley() {
        pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyLeft.setPower(-0.5);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(-0.5);
    }

    public void manualSetWristPosition (double position) {
        wrist.setPosition(position);
    }
    public void manualSetClawPosition (double position) {
        claw.setPosition(position);
    }
    public void manualSetShoulderPosition (double position) {
        shoulderL.setPosition(position);
        shoulderR.setPosition(position);
    }
    public void manualSetElbowPosition (double position) {
        elbow.setPosition(position);
    }
    public void setArmPosition(armPosition targetArmPosition){
        switch (targetArmPosition) {
            case UP:
                manualSetShoulderPosition(0);
                manualSetElbowPosition(0);
                break;
            case DOWN:
                manualSetShoulderPosition(0);
                manualSetElbowPosition(0);
                break;
        }
    }
    public void setWristPosition(wristPosition targetWristPosition){
        switch (targetWristPosition) {
            case ONE:
                manualSetWristPosition(0);
                break;
            case TWO:
                manualSetWristPosition(0);
                break;
            case THREE:
                manualSetWristPosition(0);
                break;
        }
    }
    public void setClawPosition(clawPosition targetClawPosition){
        switch (targetClawPosition) {
            case OPEN:
                manualSetClawPosition(0);
                break;
            case CLOSED:
                manualSetClawPosition(0);
                break;
        }
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


    public double PulleyPowerL() {
        return pulleyLeft.getPower();
    }
    public double PulleyPowerR() {
        return pulleyRight.getPower();
    }
}
