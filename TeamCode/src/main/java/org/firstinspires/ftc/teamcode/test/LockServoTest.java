package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@TeleOp(group = "Test")
public class LockServoTest extends LinearOpMode {
    private Servo lockServo;
    private double targetLock = 0.6;
    private Servo hoodServo;
    private double targetHood = 0.35;
    private GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        lockServo = hardwareMap.get(Servo.class, "lockServo");
        lockServo.setPosition(0.6);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(0.35);

        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            gamepad.update();

            if (gamepad.wasJustPressed(GamepadEx.Button.dpad_left)) {
                targetLock -= 0.005;
            }
            if (gamepad.wasJustPressed(GamepadEx.Button.dpad_right)) {
                targetLock += 0.005;
            }

            if (gamepad.wasJustPressed(GamepadEx.Button.dpad_up)) {
                targetHood -= 0.005;
            }
            if (gamepad.wasJustPressed(GamepadEx.Button.dpad_down)) {
                targetHood += 0.005;
            }

            lockServo.setPosition(targetLock);
            hoodServo.setPosition(targetHood);

            telemetry.addData("hoodPos: ", targetHood);
            telemetry.addData("lockPos: ", targetLock);
            telemetry.update();
        }
    }
}
