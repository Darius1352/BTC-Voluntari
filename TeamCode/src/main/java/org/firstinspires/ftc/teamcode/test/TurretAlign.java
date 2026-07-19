/*package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TURRET_ALIGNMENT", group = "Test")
public class TurretAlign extends LinearOpMode {
    Servo leftTurretServo;
    Servo rightTurretServo;

    @Override
    public void runOpMode() throws InterruptedException {
        leftTurretServo = hardwareMap.get(Servo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(Servo.class, "rightTurretServo");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            leftTurretServo.setPosition(0.5);
            rightTurretServo.setPosition(0.5);

            telemetry.addData("actualPosLeft: ", leftTurretServo.getPosition());
            telemetry.addData("actualPosRight: ", rightTurretServo.getPosition());
            telemetry.update();
        }
    }
}
 */