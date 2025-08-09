package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class exLeds {
    public final RevBlinkinLedDriver lights;
    public ledStates currentState;

    public enum ledStates {
        INIT,
        ALLIANCE_RED,
        ALLIANCE_BLUE,
        GRAB_YELLOW,
        GRAB_BLUE,
        GRAB_RED,
    }

    public exLeds(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        currentState = ledStates.INIT;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    public void setLed(ledStates state) {
        switch (state) {
            case INIT:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                currentState = state;
                break;
            case ALLIANCE_RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                currentState = state;
                break;
            case ALLIANCE_BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                currentState = state;
                break;
            case GRAB_YELLOW:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                currentState = state;
                break;
            case GRAB_BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                currentState = state;
                break;
            case GRAB_RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                currentState = state;
                break;
        }
    }
}