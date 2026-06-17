package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Continuous_Servo_Test")
public class CRservoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo upLeftLiftServo = hardwareMap.get(CRServo.class, "upLeftLiftServo");
        CRServo downLeftLiftServo = hardwareMap.get(CRServo.class, "downLeftLiftServo");
        CRServo upRightLiftServo = hardwareMap.get(CRServo.class, "upRightLiftServo");
        CRServo downRightLiftServo = hardwareMap.get(CRServo.class, "downRightLiftServo");

        upLeftLiftServo.setDirection(CRServo.Direction.FORWARD);
        downLeftLiftServo.setDirection(CRServo.Direction.FORWARD);
        upRightLiftServo.setDirection(CRServo.Direction.FORWARD);
        downRightLiftServo.setDirection(CRServo.Direction.FORWARD);

        if (gamepad1.square) {
            upLeftLiftServo.setPower(0.5);
            downLeftLiftServo.setPower(-0.5);
            telemetry.addData("Running Motor: ", "leftServos");
        } else {
            upLeftLiftServo.setPower(0);
            downLeftLiftServo.setPower(0);
        }

        if (gamepad1.circle) {
            upRightLiftServo.setPower(0.5);
            downRightLiftServo.setPower(-0.5);
            telemetry.addData("Running Motor: ", "rightServos");
        } else {
            upRightLiftServo.setPower(0);
            downRightLiftServo.setPower(0);
        }
        
    }
}
