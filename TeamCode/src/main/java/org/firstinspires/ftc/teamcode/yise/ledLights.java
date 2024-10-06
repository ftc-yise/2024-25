package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ledLights {
    public final RevBlinkinLedDriver lights;
    public ledStates currentState;



    public enum ledStates {
        INIT,
        ARM_READY,
        BLOCK_GRABBED,
        ENDGAME,

    }

    public ledLights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        currentState = ledStates.INIT;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    public void setLed(ledStates state) {
        switch (state) {
            case ENDGAME:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                currentState = state;
                break;
            case ARM_READY:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                currentState = state;
                break;
            case BLOCK_GRABBED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                currentState = state;
                break;
            case INIT:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                currentState = state;
                break;
        }
    }
}