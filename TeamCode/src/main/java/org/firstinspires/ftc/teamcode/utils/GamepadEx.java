package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {
    public Gamepad gamepad;
    public enum Button {
        touchpad_finger_1   (0x20000),
        touchpad_finger_2   (0x10000),
        touchpad            (0x08000),
        left_stick_button   (0x04000),
        right_stick_button  (0x02000),
        dpad_up             (0x01000),
        dpad_down           (0x00800),
        dpad_left           (0x00400),
        dpad_right          (0x00200),
        a                   (0x00100),
        cross               (0x00100),
        b                   (0x00080),
        circle              (0x00080),
        x                   (0x00040),
        square              (0x00040),
        y                   (0x00020),
        triangle            (0x00020),
        guide               (0x00010),
        ps                  (0x00010),
        start               (0x00008),
        options             (0x00008),
        back                (0x00004),
        share               (0x00004),
        left_bumper         (0x00002),
        right_bumper        (0x00001);

        public int integer;

        Button(int integer){
            this.integer = integer;
        }
    }
    public int currentPressed;
    private int lastPressed;

    public GamepadEx(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public boolean wasJustPressed(Button button){
        return ( ~lastPressed & currentPressed & button.integer) != 0;
    }

    public boolean isHeld(Button button) {
        return (currentPressed & button.integer) != 0;
    }

    public boolean wasJustReleased(Button button) {
        return (lastPressed & ~currentPressed & button.integer) != 0;
    }

    public void update(){
        lastPressed = currentPressed;
        currentPressed = 0;
        currentPressed = (currentPressed << 1) + (gamepad.touchpad_finger_1 ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.touchpad_finger_2 ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.touchpad ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.left_stick_button ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.right_stick_button ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.dpad_up ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.dpad_down ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.dpad_left ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.dpad_right ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.a ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.b ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.x ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.y ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.guide ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.start ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.back ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.left_bumper ? 1 : 0);
        currentPressed = (currentPressed << 1) + (gamepad.right_bumper ? 1 : 0);
    }
}