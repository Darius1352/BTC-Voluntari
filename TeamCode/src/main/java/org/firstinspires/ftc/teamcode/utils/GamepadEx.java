package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {
    public Gamepad gamepad;

    public enum Button {
        touchpad_finger_1   (0x40000), // Bit 18
        touchpad_finger_2   (0x20000), // Bit 17
        touchpad            (0x10000), // Bit 16
        left_stick_button   (0x08000), // Bit 15
        right_stick_button  (0x04000), // Bit 14
        dpad_up             (0x02000), // Bit 13
        dpad_down           (0x01000), // Bit 12
        dpad_left           (0x00800), // Bit 11
        dpad_right          (0x00400), // Bit 10
        a                   (0x00200), // Bit 9
        cross               (0x00200), // Aliased
        b                   (0x00100), // Bit 8
        circle              (0x00100), // Aliased
        x                   (0x00080), // Bit 7
        square              (0x00080), // Aliased
        y                   (0x00040), // Bit 6
        triangle            (0x00040), // Aliased
        guide               (0x00020), // Bit 5
        ps                  (0x00020), // Aliased
        start               (0x00010), // Bit 4
        options             (0x00010), // Aliased
        back                (0x00008), // Bit 3
        share               (0x00008), // Aliased
        left_bumper         (0x00002), // Bit 1
        right_bumper        (0x00001); // Bit 0

        public final int integer;

        Button(int integer){
            this.integer = integer;
        }
    }

    public int currentPressed;
    private int lastPressed;

    public GamepadEx(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public boolean isHeld(Button button){
        return (currentPressed & button.integer) != 0;
    }

    public boolean wasJustPressed(Button button){
        return (~lastPressed & currentPressed & button.integer) != 0;
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