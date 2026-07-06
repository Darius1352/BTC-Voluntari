package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@TeleOp(name = "TURRET_LIMIT_TEST", group = "Test")
public class TurretTest extends LinearOpMode {
    double TOTAL_SERVO_RANGE_DEGREES = 360;
    double TURRET_CENTER_POS = 0.5;
    double testAngle = 0;
    Servo leftTurretServo;
    Servo rightTurretServo;

    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        leftTurretServo = hardwareMap.get(Servo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(Servo.class, "rightTurretServo");
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            while(gamepad.isHeld(GamepadEx.Button.dpad_left)) {
                testAngle -= 2.5;
            }
            while(gamepad.isHeld(GamepadEx.Button.dpad_right)) {
                testAngle += 2.5;
            }

            leftTurretServo.setPosition(angleToServoPos(testAngle));
            rightTurretServo.setPosition(angleToServoPos(testAngle));

            telemetry.addData("targetAngle: ", testAngle);
            telemetry.addData("servoPos: ", angleToServoPos(testAngle));
        }

    }

    public double angleToServoPos(double angleDegrees) {
        return Math.max(0.0, Math.min(1.0, TURRET_CENTER_POS + (angleDegrees / TOTAL_SERVO_RANGE_DEGREES)));
    }
}
