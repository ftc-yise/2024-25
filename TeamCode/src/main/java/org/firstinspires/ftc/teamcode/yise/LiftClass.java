package org.firstinspires.ftc.teamcode.yise;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;

public class LiftClass {

    //define the "step" variable for the arm movements
    private int step = -1;

    //define if the dpad is currently pressed ensuring we don't run through the scoring switch
    // functions repeatedly
    public boolean buttonPressed = false;

    //creating a boolean for the hang so we can lock the hang into place
    public boolean hang = false;

    //creating a boolean to define whether or not we need to come down a certain
    //way to score for the submersible
    public boolean submersibleScoringPosition = false;

    //storage to view what the function enum is currently set too
    private pulleyPosition currentPulleyPosition;
    private liftPosition currentLiftPosition;
    private armPosition currentArmMovement;

    // double for the power of the lift
    public double liftMotorPower;

    // Arm Definitions
    public DcMotor liftLeft, liftRight, pulleyLeft, pulleyRight;
    public Servo claw, wrist, shoulderL, ShoulderR, elbow, cameraLift;
    public CRServo intake;
    public DigitalChannel limit;

    // define enum for different lift positions
    public enum liftPosition {
        HOME,
        SUBMERSIBLE,
        BASKET,
        HANG,
        AUTO_BASKET
    }

    // define enum for different Pulley positions
    public enum pulleyPosition {
        SUBMERSIBLE,
        BASKET,
        SEARCH,
        HANG,
        HANGEND,
        HOME,
        HOMEFIRST
    }

    //define global enums for hold power logic
    public enum movementState {
        REST,
        MOVING
    }

    public enum holdPowerState {
        HANG,
        SEARCH,
        SUBMERSIBLE,
        BASKET,
        HOME
    }

    public holdPowerState currentHoldPowerState;
    public movementState currentMovementState;

    //button pressed enum state
    public enum buttonPressedState {
        REST,
        HANG,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        MANUAL_UP,
        MANUAL_DOWN
    }
    public buttonPressedState currentButtonPressedState;




    // define enum for different Arm positions
    public enum armPosition {
        HOME,
        AUTOPARK,
        SUBMERSIBLESTART,
        SUBMERSIBLEEND,
        BASKET,
        SEARCH,
        HANG
    }

    // Constructor
    public LiftClass(HardwareMap hardwareMap) {
        RoadRunnerDriving drive = new RoadRunnerDriving(hardwareMap);

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

        limit = hardwareMap.get(DigitalChannel.class, "limit");

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

        //set zero power behavior for the motors
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pulleyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pulleyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // initialization pose for Driver Control and Auto
        //ToDO make the Servos & motor not move on Initiation and instead at the very start of Auto and Drive
        // control to save drivers time
        setShoulderPosition(0);
        setElbowPosition(0);
        setWristPosition(0);
        setClawPosition(0);
        setCameraHeightLOW();

        //init states for hold power
        currentHoldPowerState = holdPowerState.HOME;
        currentMovementState = movementState.REST;
        currentButtonPressedState = buttonPressedState.REST;

        // intilization making sure we arnt saving bad variables between opmodes
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
                liftMotorPower = 1;
                break;
            case AUTO_BASKET:
                liftLeft.setTargetPosition(495);
                liftRight.setTargetPosition(495);
                liftMotorPower = 0.45;
                break;
            case HOME:
                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                liftMotorPower = 0.35;
                break;
            case SUBMERSIBLE:
                liftLeft.setTargetPosition(190);
                liftRight.setTargetPosition(190);
                liftMotorPower = 1;
                break;
            case HANG:
                liftLeft.setTargetPosition(380);
                liftRight.setTargetPosition(380);
                liftMotorPower = 0.45;
                break;

        }
        // Run motors to position and define a power level
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(liftMotorPower);
        liftRight.setPower(liftMotorPower);
    }

    public void setPulleyPosition(pulleyPosition targetPulleyPosition) {
        // Stores the current position
        this.currentPulleyPosition = targetPulleyPosition;

        //switches pulleys destined position based on the target pulley position ENUM variable
        switch (targetPulleyPosition) {
            case HOMEFIRST:
                pulleyLeft.setTargetPosition(-50);
                pulleyRight.setTargetPosition(-50);
                break;
            case HOME:
                pulleyLeft.setTargetPosition(-50);
                pulleyRight.setTargetPosition(-50);
                break;
            case BASKET:
                pulleyLeft.setTargetPosition(3900);
                pulleyRight.setTargetPosition(3900);
                break;
            case SUBMERSIBLE:
                pulleyLeft.setTargetPosition(1050);
                pulleyRight.setTargetPosition(1050);
                break;
            case SEARCH:
                pulleyLeft.setTargetPosition(1650);
                pulleyRight.setTargetPosition(1650);
                break;

            case HANG:
                pulleyLeft.setTargetPosition(2550);
                pulleyRight.setTargetPosition(2550);
                break;

            case HANGEND:
                pulleyLeft.setTargetPosition(800);
                pulleyRight.setTargetPosition(800);
                break;
        }
        // Run motors to position and define a power level
        pulleyLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleyLeft.setPower(1);
        pulleyRight.setPower(1);
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
                        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        pulleyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        pulleyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                        currentHoldPowerState = holdPowerState.HANG;
                        setShoulderPosition(1);
                        setElbowPosition(0.2);

                        setLiftPosition(liftPosition.HANG);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        setPulleyPosition(pulleyPosition.HANG);
                        if (getPulleyPositionL() >= 2550) {
                            step++;
                        }
                        break;
                    case 1:
                        setLiftPower(0.5);
                        if (getLiftPositionL() >= 240) { // Replace with your own position checking logic
                            setPulleyPower(0);
                            step++;
                        }
                        break;
                    case 2:

                        sleep(550);
                        setPulleyPower(-1);
                        sleep(650);
                        setLiftPower(-0.65);
                        if (getPulleyPositionL() <= 650) {
                            setPulleyPower(-.3);
                            step++;
                        }
                        break;
                    case 3:
                        setLiftPower(-0.35);
                        setPulleyPower(-0.35);
                        if (getPulleyPositionL() <= 450) {
                            step++;
                        }
                        break;
                    case 4:
                        setPulleyPower(-.25);
                        setLiftPower(-.55);

                        currentMovementState = movementState.REST;
                        hang = true;
                        break;
                }
                break;


            case HOME:

                //case to move back into "home" or default driving position
                switch (step) {
                    case -1:  // Initialize the
                        currentHoldPowerState = holdPowerState.HOME;
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        if (!buttonPressed) {
                            setPulleyPosition(pulleyPosition.HOMEFIRST);
                            buttonPressed = true;
                        }
                        if (getPulleyPositionL() <= 250) {
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
                        setPulleyPosition(pulleyPosition.HOME);
                        if (getPulleyPositionL() <= 400) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        submersibleScoringPosition = false;
                        currentButtonPressedState = buttonPressedState.REST;
                        currentMovementState = movementState.REST;
                        step = -1;
                        break;
                }
                break;

            // case for moving from the start position of arm to the scoring position of the
            // high chamber
            case SUBMERSIBLESTART:
                 if (Parameters.attachment == Parameters.AttachmentMethod.INTERNAL) {
                        switch (step) {
                            case -1:  // Initialize the
                                currentHoldPowerState = holdPowerState.SUBMERSIBLE;
                                setWristPosition(1);
                                setShoulderPosition(0.5);
                                setElbowPosition(0.6);
                                step = 0;
                                break;
                            case 0:  // Initialize the
                                if (!buttonPressed) {
                                    setPulleyPosition(pulleyPosition.HOME);
                                }
                                if (getPulleyPositionL() <= 50) {
                                    buttonPressed = true;
                                    step++;
                                }
                                break;
                            case 1:
                                setLiftPosition(LiftClass.liftPosition.BASKET);
                                if (getLiftPositionL() >= 400) { // Replace with your own position checking logic
                                    step++;
                                }
                                break;
                            case 2:
                                setShoulderPosition(0.5);
                                setElbowPosition(0.2);
                                step = -1;
                                submersibleScoringPosition = true;
                                currentButtonPressedState = buttonPressedState.REST;
                                currentMovementState = movementState.REST;
                                break;
                        }
                } else {
                switch (step) {
                    case -1:  // Initialize the
                        currentHoldPowerState = holdPowerState.SUBMERSIBLE;
                        setShoulderPosition(0.65);
                        setElbowPosition(0.65);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        if (!buttonPressed) {
                            setPulleyPosition(pulleyPosition.HOME);
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
                        setPulleyPosition(pulleyPosition.SUBMERSIBLE);
                        if (getPulleyPositionL() >= 900) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(0.5);
                        setElbowPosition(0.5);
                        step++;
                        break;
                    case 4:
                        step = -1;
                        submersibleScoringPosition = true;
                        currentButtonPressedState = buttonPressedState.REST;
                        currentMovementState = movementState.REST;
                        break;
                }
            }
                break;

            // case for moving from the scoring position of the high chamber back to the home
            // position
            case SUBMERSIBLEEND:
                 if (Parameters.attachment == Parameters.AttachmentMethod.INTERNAL) {
                    switch (step) {
                        case -1:  // Initialize the
                            currentHoldPowerState = holdPowerState.HOME;
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
                            submersibleScoringPosition = false;
                            currentButtonPressedState = buttonPressedState.REST;
                            currentMovementState = movementState.REST;
                            break;
                    }
                }else {
                switch (step) {
                    case -1:  // Initialize the
                        currentHoldPowerState = holdPowerState.HOME;
                        setShoulderPosition(0.25);
                        setElbowPosition(0.5);
                        step = 0;
                        break;
                    case 0:
                        setPulleyPosition(pulleyPosition.HOME);
                            step++;
                    case 1:
                        sleep(350);
                        setClawPosition(1);
                        step++;
                        break;
                    case 2:
                        setWristPosition(0);
                        setLiftPosition(LiftClass.liftPosition.HOME);
                        step = -1;
                        submersibleScoringPosition = false;
                        currentButtonPressedState = buttonPressedState.REST;
                        currentMovementState = movementState.REST;
                        break;
                }
            }
                break;

            //case to move into basket searching position
            case BASKET:
                switch (step) {
                    case -1:  // Initialize the
                        currentHoldPowerState = holdPowerState.BASKET;
                        setShoulderPosition(0.6);
                        setElbowPosition(0.02);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        if (!buttonPressed) {
                            setPulleyPosition(pulleyPosition.HOME);
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
                        setPulleyPosition(pulleyPosition.BASKET);
                        if (getPulleyPositionL() >= 3000) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(1);
                        setElbowPosition(0.6);
                        step = -1;
                        submersibleScoringPosition = false;
                        currentButtonPressedState = buttonPressedState.REST;
                        currentMovementState = movementState.REST;
                        break;
                }
                break;

                //case to move into submersible searching position
            case SEARCH:
                switch (step) {
                    case -1:  // Initialize the
                        currentHoldPowerState = holdPowerState.SEARCH;
                        setShoulderPosition(0.25);
                        setElbowPosition(0);
                        step = 0;
                        break;
                    case 0:  // Initialize the
                        if (!buttonPressed) {
                            setPulleyPosition(pulleyPosition.HOME);
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
                        setPulleyPosition(pulleyPosition.SEARCH);
                        if (getPulleyPositionL() >= 1500) { // Replace with your own position checking logic
                            step++;
                        }
                        break;
                    case 3:
                        setShoulderPosition(0.4);
                        setElbowPosition(0);
                        step = -1;
                        submersibleScoringPosition = false;
                        currentButtonPressedState = buttonPressedState.REST;
                        currentMovementState = movementState.REST;
                        break;
                }
                break;

            //case to move into parking position for level one hang in auto
            case AUTOPARK:
                switch (step) {
                    case -1:  // Initialize the
                        setShoulderPosition(1);
                        setElbowPosition(0.5);
                        step = 0;
                        break;
                    case 0:
                        manualPowerUpLift();
                        setLiftPosition(liftPosition.SUBMERSIBLE);
                            step++;
                        break;
                    case 2:
                        setShoulderPosition(0.25);
                        setElbowPosition(0.5);
                        sleep(1000);
                        step = -1;
                        submersibleScoringPosition = false;
                        currentButtonPressedState = buttonPressedState.REST;
                        currentMovementState = movementState.REST;
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
            liftLeft.setPower(0.16);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.16);
        }
    }

    public void zeroPowerPulley() {
        if (!pulleyLeft.isBusy() && !pulleyRight.isBusy()) {
            pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyLeft.setPower(0.04);
            pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyRight.setPower(0.04);
        }
    }

    // power functions designed to hold both lift and pulley at a set position by running minimal
    // power through the motors
    public void neutralPowerLift() {
        if (!liftLeft.isBusy() && !liftRight.isBusy()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(0.0);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.0);
        }
    }

    public void neutralPowerPulley() {
        if (!pulleyLeft.isBusy() && !pulleyRight.isBusy()) {
            pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyLeft.getZeroPowerBehavior();
            pulleyLeft.setPower(0.0);
            pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyRight.setPower(0.0);
        }
    }

    public void hangPowerPulley() {
        if (!pulleyLeft.isBusy() && !pulleyRight.isBusy()) {
            pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyLeft.setPower(-0.15);
            pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pulleyRight.setPower(-0.15);
        }
    }

    public void hangPowerLift() {
        if (!liftLeft.isBusy() && !liftRight.isBusy()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(0.1);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setPower(0.1);
        }
    }

    // setting power to both lift to maneuver them without the use of motor encoders
    public void manualPowerUpLift() {
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(0.5);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setPower(0.5);
    }

    public void manualPowerDownLift() {
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(-0.35);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setPower(-0.35);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // setting power to both pulley to maneuver them without the use of motor encoders
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
        pulleyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyLeft.setPower(power);
        pulleyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyRight.setPower(power);
    }

    public void setLiftPower(double power) {
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(power);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setPower(power);
    }

    public void setWristPosition(double power) {
        wrist.setPosition(power);
    }

    public void CloseClaw() {
        claw.setPosition(0);
    }

    public void OpenClaw() {
        claw.setPosition(1);
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

    public pulleyPosition getCurrentPulleyPosition() {
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

    // getter statements for the 2 boolean switch program statements

    public boolean getSubmersibleScoringPosition(){
        return submersibleScoringPosition;
    }

    public boolean getButtonPressed() {
        return buttonPressed;
    }

    public boolean getHang() {
        return hang;
    }
}
