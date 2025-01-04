package org.firstinspires.ftc.teamcode.yise;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftClass {

    //define the "step" variable for the arm movements
    private int step = -1;

    //define if the dpad is currently pressed ensuring we don't run through the scoring switch
    // functions repeatedly
    public static boolean buttonPressed;

    // determining if a dpad has been hit so we don't run throw loops repeatedly and we dont run
    // through two loops at once
    public static Boolean dpadUptapped = false;
    public static Boolean dpadDowntapped = false;
    public static Boolean dpadLefttapped = false;
    public static Boolean dpadRighttapped = false;

    //creating a boolean for the hang so we can lock the hang into place
    public static Boolean hang = false;
    public static Boolean pulleyHold = false;

    //storage to view what the function enum is currently set too
    private PulleyPosition currentPulleyPosition;
    private liftPosition currentLiftPosition;
    private armPosition currentArmMovement;

    // power for the right pulley since it is now chained to the left encoder
    public double pulleyRightPower;


    // Arm Definitions
    public DcMotor liftLeft, liftRight, pulleyLeft, pulleyRight;
    public Servo claw, wrist, shoulderL, ShoulderR, elbow, cameraLift;
    public CRServo intake;

    // define enum for different lift positions
    public enum liftPosition {
        HOME,
        SUBMERSIBLE,
        BASKET,
        HANG
    }

    // define enum for different Pulley positions
    public enum PulleyPosition {
        HOME,
        SUBMERSIBLE,
        BASKET,
        SEARCH,
        HANG,
        HANGEND
    }

    // define enum for different Arm positions
    public enum armPosition {
        HOME,
        SUBMERSIBLESTART,
        SUBMERSIBLEEND,
        BASKET,
        SEARCH,
        HANG
    }


    public double armMotorPower;

    // Constructor
    public LiftClass(HardwareMap hardwareMap) {
        //Initialize arm motors
        liftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        liftRight = hardwareMap.get(DcMotor.class, "liftright");

        pulleyLeft = hardwareMap.get(DcMotor.class, "pulleyLeft");
        pulleyRight = hardwareMap.get(DcMotor.class, "pulleyRight");

        //Initialize arm Servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        ShoulderR = hardwareMap.get(Servo.class, "shoulderR");
        elbow = hardwareMap.get(Servo.class, "elbow");

        cameraLift = hardwareMap.get(Servo.class, "cameraLift");

        intake = hardwareMap.get(CRServo.class, "intake");

        //Set motor and servo directions
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        pulleyLeft.setDirection(DcMotor.Direction.REVERSE);
        shoulderL.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(CRServo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        //Reset motor encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pulleyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleyRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // initialization pose for Driver Control and Auto
        //ToDO make the Servos & motor not move on Initiation and instead at the very start of Auto and Drive
        // control to save drivers time
        setShoulderPosition(0.25);
        setElbowPosition(0.5);
        setWristPosition(0);
        setClawPosition(0);
        setCameraHeightLOW();

        // intilization making sure we arnt saving bad variables between opmodes
        pulleyHold = false;
        hang = false;
    }

    public void setLiftPosition(liftPosition targetLiftPosition) {
        // Stores the current position
        this.currentLiftPosition = targetLiftPosition;
        //switches the lifts destined position based on the target lift position ENUM variable
        switch (targetLiftPosition) {
            case BASKET:
                liftLeft.setTargetPosition(495);
                liftRight.setTargetPosition(495);
                armMotorPower = 100;
                break;
            case HOME:
                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                armMotorPower = 0.45;
                break;
            case SUBMERSIBLE:
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
        // Run motors to position and define a power level
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(armMotorPower);
        liftRight.setPower(armMotorPower);
    }

    public void setPulleyPosition(LiftClass.PulleyPosition targetPulleyPosition) {
        // Stores the current position
        this.currentPulleyPosition = targetPulleyPosition;

        //switches pulleys destined position based on the target pulley position ENUM variable
        switch (targetPulleyPosition) {
            case HOME:
                pulleyLeft.setTargetPosition(0);
                pulleyRight.setTargetPosition(0);
                pulleyRightPower = -1;
                break;
            case BASKET:
                pulleyLeft.setTargetPosition(3800);
                pulleyRight.setTargetPosition(3800);
                pulleyRightPower = 1;
                break;
            case SUBMERSIBLE:
                pulleyLeft.setTargetPosition(1450);
                pulleyRight.setTargetPosition(1450);
                pulleyRightPower = 1;
                break;
            case SEARCH:
                pulleyLeft.setTargetPosition(2000);
                pulleyRight.setTargetPosition(2000);
                pulleyRightPower = 1;
                break;

            case HANG:
                pulleyLeft.setTargetPosition(2900);
                pulleyRight.setTargetPosition(2900);
                pulleyRightPower = 1;
                break;

            case HANGEND:
                pulleyLeft.setTargetPosition(800);
                pulleyRight.setTargetPosition(800);
                pulleyRightPower = 1;
                break;
        }
        // Run motors to position and define a power level
        pulleyLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulleyLeft.setPower(1);
        pulleyRight.setPower(pulleyRightPower);
    }

    //switch Case statement for moving to High Basket Scoring position
    public void setArmPosition(LiftClass.armPosition armMovement) throws InterruptedException {
        // Stores the current position
        this.currentArmMovement = armMovement;

        //switches pulleys destined position based on the target pulley position ENUM variable
        switch (armMovement) {
            case HANG:
                switch (step) {
                    case -1:  // Initialize the
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        setPulleyPosition(LiftClass.PulleyPosition.HANG);
                        if (getPulleyPositionL() >= 2400) {
                            step++;
                        }
                        break;
                    case 1:
                        setLiftPosition(LiftClass.liftPosition.HANG);
                        if (getLiftPositionL() >= 150) { // Replace with your own position checking logic
                            sleep(2000);
                            step++;
                        }
                        break;
                    case 2:
                        setPulleyPosition(LiftClass.PulleyPosition.HOME);
                        if (getPulleyPositionL() <= 150) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(1);
                        setElbowPosition(0.65);
                        step++;
                        break;

                    case 4:
                        hang = true;
                        setPulleyPosition(LiftClass.PulleyPosition.HANGEND);
                        if (getPulleyPositionL() >= 600) { // Replace with your own position checking logic
                            sleep(2000);
                            step++;
                        }
                        break;
                    case 5:
                        hangPowerPulley();
                }
            break;

            case HOME:

                //case to move back into "home" or default driving position
                switch (step) {
                    case -1:  // Initialize the
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        if (!buttonPressed) {
                            setPulleyPosition(LiftClass.PulleyPosition.HOME);
                            buttonPressed = true;
                        }
                        if (getPulleyPositionL() <= 350) {
                            step++;
                        }
                        break;
                    case 1:
                        setLiftPosition(LiftClass.liftPosition.HOME);
                        if (getLiftPositionL() <= 100) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 2:
                        setPulleyPosition(LiftClass.PulleyPosition.HOME);
                        if (getPulleyPositionL() <= 400) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        step = -1;
                        dpadDowntapped = false;
                        break;
                }
                break;

            // case for moving from the start position of arm to the scoring position of the
            // high chamber
            case SUBMERSIBLESTART:
                if (Parameters.attachment == Parameters.AttachmentMethod.OVERTHETOP){
                    switch (step) {
                        case -1:  // Initialize the
                            setShoulderPosition(0.25);
                            setElbowPosition(0);
                            step = 0;
                            break;
                        case 0:  // Initialize the
                            if (!buttonPressed) {
                                setPulleyPosition(LiftClass.PulleyPosition.HOME);
                                buttonPressed = true;
                            }
                            if (getPulleyPositionL() <= 50) {
                                step++;
                            }
                            break;
                        case 1:
                            setLiftPosition(LiftClass.liftPosition.BASKET);
                            if (getLiftPositionL() >= 400) { // Replace with your own position checking logic
                                setWristPosition(1);
                                step++;
                            }
                            break;
                        case 2:
                            setPulleyPosition(LiftClass.PulleyPosition.SUBMERSIBLE);
                            if (getPulleyPositionL() >= 1300) { // Replace with your own position checking logic
                                step++;
                            }
                            break;
                        case 3:
                            setShoulderPosition(0.3);
                            setElbowPosition(0.5);
                            step++;
                            break;
                        case 4:
                            step = -1;
                            dpadRighttapped = false;
                            break;
                    }
                } else {
                        switch (step) {
                            case -1:  // Initialize the
                                setWristPosition(1);
                                setShoulderPosition(0.5);
                                setElbowPosition(0.5);
                                step = 0;
                                break;
                            case 0:  // Initialize the
                                if (!buttonPressed) {
                                    setPulleyPosition(LiftClass.PulleyPosition.HOME);
                                }
                                if (getPulleyPositionL() <= 50) {
                                    buttonPressed = true;
                                    step++;
                                }
                                break;
                            case 1:
                                setClawPosition(0.5);
                                setLiftPosition(LiftClass.liftPosition.BASKET);
                                if (getLiftPositionL() >= 400) { // Replace with your own position checking logic
                                    step++;
                                }
                                break;
                            case 2:
                                sleep(250);
                                setClawPosition(0.25);
                                setShoulderPosition(0.5);
                                setElbowPosition(0.2);
                                step = -1;
                                dpadRighttapped = false;
                                break;
                        }
                }
                break;

            // case for moving from the scoring position of the high chamber back to the home
            // position
            case SUBMERSIBLEEND:
                if (Parameters.attachment == Parameters.AttachmentMethod.OVERTHETOP){
                    switch (step) {
                        case -1:  // Initialize the
                            setShoulderPosition(0.3);
                            setElbowPosition(0.5);
                            step = 0;
                            break;
                        case 0:
                            setPulleyPosition(LiftClass.PulleyPosition.HOME);
                            if (getPulleyPositionL() <= 250) {
                                step++;
                            }
                        case 1:
                            sleep(460);
                            setClawPosition(1);
                            step++;
                            break;
                        case 2:
                            setWristPosition(0);
                            setLiftPosition(LiftClass.liftPosition.HOME);
                            step = -1;
                            dpadRighttapped = false;
                            break;
                    }
                } else {
                    switch (step) {
                        case -1:  // Initialize the
                            setShoulderPosition(0.45);
                            setElbowPosition(0.2);
                            step = 0;
                            break;
                        case 0:
                            sleep(150);
                            setClawPosition(1);
                            sleep(150);
                            step++;
                            break;
                        case 1:
                            setLiftPosition(LiftClass.liftPosition.HOME);
                            step = -1;
                            dpadRighttapped = false;
                            break;
                    }
                }
                break;

            //case to move into basket searching position
            case BASKET:
                switch (step) {
                    case -1:  // Initialize the
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        if (!buttonPressed) {
                            setPulleyPosition(LiftClass.PulleyPosition.HOME);
                            buttonPressed = true;
                        }
                        if (getPulleyPositionL() <= 350) {
                            step++;
                        }
                        break;
                    case 1:
                        setLiftPosition(LiftClass.liftPosition.BASKET);
                        if (getLiftPositionL() >= 300) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 2:
                        setPulleyPosition(LiftClass.PulleyPosition.BASKET);
                        if (getPulleyPositionL() >= 3000) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(1);
                        setElbowPosition(0.65);
                        step = -1;
                        dpadUptapped = false;
                        break;
                }
                break;

                //case to move into submersible searching position
            case SEARCH:
                switch (step) {
                    case -1:  // Initialize the
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        if (!buttonPressed) {
                            setPulleyPosition(LiftClass.PulleyPosition.HOME);
                            buttonPressed = true;
                        }
                        if (getPulleyPositionL() <= 350) {
                            step++;
                        }
                        break;
                    case 1:
                        setLiftPosition(LiftClass.liftPosition.HOME);
                        if (getLiftPositionL() <= 100) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 2:
                        setPulleyPosition(LiftClass.PulleyPosition.SEARCH);
                        if (getPulleyPositionL() >= 1500) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(0.4);
                        setElbowPosition(0);
                        step = -1;
                        dpadLefttapped = false;
                        break;
                }
                break;
        }
    }

    // power functions designed to hold both lift and pulley at a set position by running minimal
    // power through the motors
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

    public void hangPowerPulley() {
        if (!pulleyLeft.isBusy() && !pulleyRight.isBusy()) {
            pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyLeft.setPower(-0.35);
            pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyRight.setPower(-0.35);
        }
    }

    // setting power to both lift and pulley to maneuver them without the use of motor encoders
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
        pulleyLeft.setPower(-0.65);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(-0.65);
        pulleyLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        pulleyRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    // set rigid and defined heights for the servo controlling the opencv webcam
    public void setCameraHeightHIGH(){
        cameraLift.setPosition(0);
    }
    public void setCameraHeightLOW(){
        cameraLift.setPosition(0.35);
    }

    // setting custom positions and power for servos
    public void setPulleyPower(double power) {
        pulleyLeft.setPower(power);
        pulleyRight.setPower(power);
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

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    //writer/setter statements for the seven arm movement booleans
    public void setHangStatus(boolean status) {
        hang = status;
    }

    public void setPulleyHoldStatus(boolean status){
        pulleyHold = status;
    }

    public void setDpadUpTappedStatus(boolean status) {
        dpadUptapped = status;
    }

    public void setDpadDowntappedStatus(boolean status) {
        dpadDowntapped = status;
    }

    public void setDpadLefttappedStatus(boolean status) {
        dpadLefttapped = status;
    }

    public void setDpadRighttappedStatus(boolean status) {
        dpadRighttapped = status;
    }

    public void setButtonPressedStatus(boolean status) {
        buttonPressed = status;
    }

    // Getter Position functions for telemetry debugging
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

    public PulleyPosition getCurrentPulleyPosition() {
        return currentPulleyPosition; // Return the stored current position
    }

    public liftPosition getCurrentLiftPosition() {
        return currentLiftPosition; // Return the stored current position
    }

    public armPosition getCurrentArmMovement() {
        return currentArmMovement; // Return the stored current position
    }

    // Getter Power functions for telemetry debugging
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

    // getter statements for the 7 boolean switch program statements
    public boolean getHangStatus() {
        return hang;
    }

    public boolean getPulleyHoldStatus(){
        return pulleyHold;
    }

    public boolean getDpadUpTapped() {
        return dpadUptapped;
    }

    public boolean getDpadDownTapped() {
        return dpadDowntapped;
    }

    public boolean getDpadLeftTapped() {
        return dpadLefttapped;
    }

    public boolean getDpadRightTapped() {
        return dpadRighttapped;
    }

    public boolean getButtonPressed() {
        return buttonPressed;
    }
}
