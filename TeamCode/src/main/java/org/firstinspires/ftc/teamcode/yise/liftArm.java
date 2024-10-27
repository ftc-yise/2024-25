package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class liftArm {
    public DcMotor armLeft, armRight;
    public enum armPosition {
        DOWN,
        UP
    }
    public armPosition currentArmPosition;

    //Constructor
    public liftArm(HardwareMap hardwareMap) {
        //Initialize motors and servos
        armLeft = hardwareMap.get(DcMotor.class, "armleft");
        armRight = hardwareMap.get(DcMotor.class, "armright");

        //Set motor directions
        armRight.setDirection(DcMotor.Direction.FORWARD);
        armLeft.setDirection(DcMotor.Direction.REVERSE);

        //Reset encoders
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setArmPosition(liftArm.armPosition.DOWN);
    }

    public void setArmPosition(liftArm.armPosition targetArmPosition) {
        switch (targetArmPosition) {
            case DOWN:
                armLeft.setTargetPosition(0);
                armRight.setTargetPosition(0);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setPower(0.25);
                armLeft.setPower(0.25);
                break;
            case UP:
                armLeft.setTargetPosition(1);
                armRight.setTargetPosition(-1);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLeft.setPower(0.25);
                armRight.setPower(0.25);
                break;
        }
        currentArmPosition = targetArmPosition;
    }



}
