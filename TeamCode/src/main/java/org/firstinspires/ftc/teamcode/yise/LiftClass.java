package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftClass {
    //storage to make sure we don't over extend ever
    private PulleyPosition currentPulleyPosition;
    private liftPosition currentLiftPosition;


    // Arm Variables
    public DcMotor liftLeft, liftRight, pulleyLeft, pulleyRight;
    public Servo claw, wrist, shoulderL, ShoulderR, elbow;
    public enum liftPosition {
        HOME,
        SUBMERSABLE,
        BASKET,
        HANG
    }

    public enum PulleyPosition {
        HOME,
        SUBMERSABLE, BASKET, SEARCH, HANG, HANGEND
    }

    public double armMotorPower;
    public double leftArmMotorPositionValue;
    public double rightArmMotorPositionValue;

    // Constructor
    public LiftClass(HardwareMap hardwareMap) {
        //Initialize arm motors
        liftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        liftRight = hardwareMap.get(DcMotor.class, "liftright");

        pulleyLeft = hardwareMap.get(DcMotor.class, "pulleyLeft");
        pulleyRight = hardwareMap.get(DcMotor.class, "pulleyRight");

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        ShoulderR = hardwareMap.get(Servo.class, "shoulderR");
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

        setShoulderPosition(0);
        setElbowPosition(0.5);
        setWristPosition(0);
        setClawPosition(0);
    }

    public void setLiftPosition(liftPosition targetLiftPosition) {
        this.currentLiftPosition = targetLiftPosition; // Store the current position
        switch (targetLiftPosition) {
            case BASKET:
                liftLeft.setTargetPosition(495);
                liftRight.setTargetPosition(495);
                armMotorPower = 100;
                break;
            case HOME:
                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                armMotorPower = 0.25;
                break;
            case SUBMERSABLE:
                liftLeft.setTargetPosition(285);
                liftRight.setTargetPosition(285);
                armMotorPower = 100;
                break;
            case HANG:
                liftLeft.setTargetPosition(265);
                liftRight.setTargetPosition(265);
                armMotorPower = 100;
                break;

        }
        // Run motors to position using defined power level
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(armMotorPower);
        liftRight.setPower(armMotorPower);
    }

    public void setPulleyPosition(LiftClass.PulleyPosition targetPulleyPosition) {
        this.currentPulleyPosition = targetPulleyPosition; // Store the current position
        switch (targetPulleyPosition) {
            case HOME:
                pulleyLeft.setTargetPosition(0);
                pulleyRight.setTargetPosition(0);
                break;
            case BASKET:
                pulleyLeft.setTargetPosition(3900);
                pulleyRight.setTargetPosition(3900);
                break;
            case SUBMERSABLE:
                pulleyLeft.setTargetPosition(2150);
                pulleyRight.setTargetPosition(2150);
                break;
            case SEARCH:
                pulleyLeft.setTargetPosition(2000);
                pulleyRight.setTargetPosition(2000);
                break;

            case HANG:
                pulleyLeft.setTargetPosition(2900);
                pulleyRight.setTargetPosition(2900);
                break;

            case HANGEND:
                pulleyLeft.setTargetPosition(800);
                pulleyRight.setTargetPosition(800);
                break;
        }
        // Run motors to position using defined power level
        pulleyLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyLeft.setPower(1);
        pulleyRight.setPower(-1);
    }

    public void zeroPowerLift() {
        if (!liftLeft.isBusy() && !liftRight.isBusy()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(0.08);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.08);
        }
    }

    public void zeroPowerPulley() {
        if (!pulleyLeft.isBusy() && !pulleyRight.isBusy()) {
            pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyLeft.setPower(0.06);
            pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyRight.setPower(0.06);
        }
    }

    public void heroPowerPulley() {
        if (!pulleyLeft.isBusy() && !pulleyRight.isBusy()) {
            pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyLeft.setPower(-0.35);
            pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyRight.setPower(-0.35);
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
        pulleyLeft.setPower(0.35);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(0.35);
    }
    public void manualPowerDownPulley() {
        pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyLeft.setPower(-0.35);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(-0.35);
    }

    public void setWristPosition(double power) {
        wrist.setPosition(power);
    }
    public void setClawPosition(double position) {
        claw.setPosition(position);
    }
    public void setShoulderPosition(double position) {
        shoulderL.setPosition(position);
        ShoulderR.setPosition(position);
    }
    public void setElbowPosition(double position) {
        elbow.setPosition(position);
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

    public double LiftPowerL() {
        return liftLeft.getPower();
    }
    public double LiftPowerR() {
        return liftRight.getPower();
    }

    public PulleyPosition getCurrentPulleyPosition() {
        return currentPulleyPosition; // Return the stored current position
    }

    public liftPosition getCurrentLiftPosition() {
        return currentLiftPosition; // Return the stored current position
    }
}
