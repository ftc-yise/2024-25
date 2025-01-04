package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ledLights {
    public final RevBlinkinLedDriver lights;
    public ledStates currentState;

    public enum ledStates {
        INIT,
        RED,
        BLUE,
        GRAB_Y,
        GRAB_B,
        GRAB_R,
        ENDGAME, CLAW_OPEN,

    }

    public ledLights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        currentState = ledStates.INIT;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    public void setLed(ledStates state) {
        switch (state) {
            case ENDGAME:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
                currentState = state;
                break;
            case GRAB_Y:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                currentState = state;
                break;
            case GRAB_B:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                currentState = state;
                break;
            case GRAB_R:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                currentState = state;
                break;
            case INIT:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                currentState = state;
                break;
            case RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
                currentState = state;
                break;
            case BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                currentState = state;
                break;
            case CLAW_OPEN:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                currentState = state;
                break;
        }
    }
}