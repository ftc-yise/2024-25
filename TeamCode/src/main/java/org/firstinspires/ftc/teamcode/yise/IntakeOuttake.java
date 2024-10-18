package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeOuttake {
    public DcMotor pulleyL, pulleyR, liftL, liftR;
    public Servo claw, clawRotation, clawLift;

    public PulleyPosition pulleyPosition;

    public LiftRotation liftRotation;

    public double intakePower = 0;

    // Used to set arm height.
    public enum PulleyPosition {
        INTAKE,
        SUBMERSIBLELOW,
        SUBMERSIBLEHIGH,
        BASKETLOW,
        BASKETHIGH,
        DEFAULT
    }

    // Used to set lift rotation.
    public enum LiftRotation {
        SUBMERSIBLELOWANGLE,
        SUBMERSIBLEHIGHANGLE,
        BASKETLOWANGLE,
        BASKETHIGHANGLE,
        DEFAULTANGLE
    }


    //Constructor
    public IntakeOuttake(HardwareMap hardwareMap) {
        //Initialize motors and servos
        pulleyL = hardwareMap.get(DcMotor.class, "leftPulley");
        pulleyR = hardwareMap.get(DcMotor.class, "rightPulley");
        liftL = hardwareMap.get(DcMotor.class, "leftLift");
        liftR = hardwareMap.get(DcMotor.class, "rightLift");

        claw = hardwareMap.get(Servo.class, "claw");
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        clawLift = hardwareMap.get(Servo.class, "clawLift");

        clawLift.setPosition(Servo.MIN_POSITION);
        clawRotation.setPosition(Servo.MIN_POSITION);
        claw.setPosition(Servo.MIN_POSITION);

        //Set motor directions
        liftL.setDirection(DcMotor.Direction.REVERSE);

        //Reset encoders
        pulleyL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleyR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    public void setPulleyPosition(PulleyPosition pulleyPosition) {
        pulleyR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulleyL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        switch (pulleyPosition) {
            case DEFAULT:
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case SUBMERSIBLELOW:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case SUBMERSIBLEHIGH:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case BASKETLOW:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case BASKETHIGH:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case INTAKE:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
        }
        pulleyL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyL.setPower(0.5);
        pulleyR.setPower(0.5);
    }

    public void manualExtend(double speed) {
        pulleyL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyL.setPower(speed);
        pulleyR.setPower(speed);
    }

    public void setLiftOrientation(LiftRotation liftRotation) {
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        switch (liftRotation) {
            case DEFAULTANGLE:
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case SUBMERSIBLELOWANGLE:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case SUBMERSIBLEHIGHANGLE:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case BASKETLOWANGLE:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
            case BASKETHIGHANGLE:
                /** Height Set To Zero Until Arm is Complete*/
                pulleyR.setTargetPosition(0);
                pulleyL.setTargetPosition(0);
                break;
        }
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(0.5);
        liftR.setPower(0.5);
    }



    public void openClaw() {
        claw.setPosition(Servo.MAX_POSITION);
    }
    public void closeTrapdoor() {
        claw.setPosition(Servo.MIN_POSITION);
    }

    public void setClawLiftPostion(double postion) {
        clawLift.setPosition(postion);
    }
    public void clawRotation(double postion) {
        clawRotation.setPosition(postion);
    }


    public void extend(PulleyPosition pulleyPosition, LiftRotation targetRotation) {
        try {
            setLiftOrientation(targetRotation);
            Thread.sleep(500);
            setPulleyPosition(pulleyPosition);
        }catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

    public void retract() {
        closeTrapdoor();
        setLiftOrientation(LiftRotation.DEFAULTANGLE);
        setPulleyPosition(PulleyPosition.DEFAULT);
    }

    public void holdPulley() {
        if (!pulleyL.isBusy()) {
            pulleyL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyR.setPower(0.05);
            pulleyL.setPower(0.05);
        }
    }

    public double getPulleyPosition() {
        return pulleyL.getCurrentPosition();
    }

    public double getLiftPosition() {
        return liftL.getCurrentPosition();
    }
}