package org.firstinspires.ftc.teamcode.Libs.Classes;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.EnumMap;

import org.firstinspires.ftc.teamcode.utils.Lambda;
import org.firstinspires.ftc.teamcode.utils.LambdaWithParam;
import org.firstinspires.ftc.teamcode.utils.Utils;


//UnDocumented Orville Code - Ew
public class GameController {
    public double rightJoyStickX() {
        return gamepad.right_stick_x;
    }

    public enum Button {
        Y, B, A, X,
        DPAD_DOWN, DPAD_UP, DPAD_LEFT, DPAD_RIGHT,

        LEFT_BUMPER, RIGHT_BUMPER,

        R3, L3,

        Start, Select, Guide, TouchPad,
        TOUCH_1, TOUCH_2,
    }
    public enum Trigger {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }
    public enum TouchPad {
        Finger1X, Finger1Y, Finger2X, Finger2Y

    }

    EnumMap<Button, Boolean> wasButtonPressed = new EnumMap<>(Button.class);
    Gamepad gamepad;

    public GameController(Gamepad gamepad) {
        this.gamepad = gamepad;
        wasButtonPressed.put(Button.X, false);
        wasButtonPressed.put(Button.Y, false);
        wasButtonPressed.put(Button.A, false);
        wasButtonPressed.put(Button.B, false);
    }

    public boolean getButtonState(Button btn) {
        switch (btn) {
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;

            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case RIGHT_BUMPER:
                return  gamepad.right_bumper;
            case LEFT_BUMPER:
                return  gamepad.left_bumper;
            case R3:
                return gamepad.right_stick_button;
            case L3:
                return  gamepad.left_stick_button;

            case Start:
                return gamepad.start;
            case Select:
                return gamepad.back;
            case Guide:
                return gamepad.guide;
            case TouchPad:
                return gamepad.touchpad;

            case TOUCH_1:
                return gamepad.touchpad_finger_1;
            case TOUCH_2:
                return gamepad.touchpad_finger_2;
        }
        throw new RuntimeException("Unexpected btn: " + btn);
    }

    public float getTouchPadState(TouchPad touchPad) {
        switch (touchPad) {
            case Finger1X:
                return gamepad.touchpad_finger_1_x;
            case Finger1Y:
                return gamepad.touchpad_finger_1_y;
            case Finger2X:
                return gamepad.touchpad_finger_2_x;
            case Finger2Y:
                return gamepad.touchpad_finger_2_y;
        }
        throw new RuntimeException("Unexpected touchPad: " + touchPad);
    }

    public void updateButtonStates() {
        for (Button btn : Button.values()) { // For each button
            wasButtonPressed.put(btn, getButtonState(btn)); // Update buttonState
        }

    }
    public void onButton(Button btn, Lambda lambda){
        if(getButtonState(btn)){
            lambda.f();
        }
    }

    /**
     * Is a better name, onButtonPressReleased or onButtonReleased??
     *
     * @param btn
     * @param lambda
     */
    public void onButtonPressReleased(Button btn, Lambda lambda) {
        if (Boolean.TRUE.equals(wasButtonPressed.get(btn)) && !getButtonState(btn)) {
            // Button was pressed and Button is not pressed now
            lambda.f(); // Run the side effect
        }

    }
    public boolean isTriggerDown(Trigger t) {
        if(t == Trigger.LEFT_TRIGGER) {
            return gamepad.left_trigger  >=0.7;
        } else if(t == Trigger.RIGHT_TRIGGER) {
            return gamepad.right_trigger >= 0.7;
        }
        return false;
    }
    public void onButtonTap(Button btn, Lambda lambda) {
        // Will run the lambda once when the button is pressed, will not rerun until the button is released and pressed again.
        if (Boolean.FALSE.equals(wasButtonPressed.get(btn)) && getButtonState(btn)) {
            // Button was not pressed and Button is pressed now
            lambda.f(); // Run the side effect
        }

    }
    public void onButtonCombinationHeld(Button[] buttons, Lambda lambda) {
        if (Utils.every(buttons, btn -> wasButtonPressed.get(btn) && getButtonState(btn))) {
            // All buttons were pressed and are pressed now
            lambda.f(); // Run the side effect
        }
    }
    public void onTrigger(Trigger trigger, LambdaWithParam<Float> lambda) {
        if (trigger == Trigger.LEFT_TRIGGER) {
            if (gamepad.left_trigger > 0.5) {
                lambda.f(gamepad.left_trigger );
            }
        } else if (trigger == Trigger.RIGHT_TRIGGER) {
            if (gamepad.right_trigger > 0.5) {
                lambda.f(gamepad.right_trigger );
            }
        }
    }
    public float leftJoyStickY(){
        return gamepad.left_stick_y;
    }
    public float leftJoyStickX(){
        return gamepad.left_stick_x;
    }
    public float rightJoyStickY(){
        return gamepad.right_stick_y;
    }
    public float rightTrigger(){
        return gamepad.right_trigger;
    }
    public float leftTrigger(){ return gamepad.left_trigger;}

    //Rumble
    public void Rumble(int time, float power){
        gamepad.rumble(power,power, time);
    }
    //LED
    public void setLed(int r, int g, int b, int Time){
        gamepad.setLedColor(r,g,b,Time);
    }

}
