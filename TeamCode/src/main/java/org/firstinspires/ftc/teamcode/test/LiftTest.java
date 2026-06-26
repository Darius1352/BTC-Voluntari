package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lift_Test", group = "Test")
public class LiftTest extends LinearOpMode {

    private CRServo upLeftLiftServo;
    private CRServo downLeftLiftServo;
    private CRServo upRightLiftServo;
    private CRServo downRightLiftServo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        upLeftLiftServo = hardwareMap.get(CRServo.class, "upLeftLiftServo");
        downLeftLiftServo = hardwareMap.get(CRServo.class, "downLeftLiftServo");
        upRightLiftServo = hardwareMap.get(CRServo.class, "upRightLiftServo");
        downRightLiftServo = hardwareMap.get(CRServo.class, "downRightLiftServo");

        telemetry.addData("Status: ", "init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double manualPower = 0.0;
            if (gamepad1.dpad_up) {
                manualPower = 0.5;
            } else if (gamepad1.dpad_down) {
                manualPower = -0.5;
            }

            upLeftLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);
            downLeftLiftServo.setDirection(DcMotorSimple.Direction.FORWARD);
            upRightLiftServo.setDirection(DcMotorSimple.Direction.FORWARD);
            downRightLiftServo.setDirection(DcMotorSimple.Direction.REVERSE);

            upLeftLiftServo.setPower(manualPower);
            downLeftLiftServo.setPower(manualPower);
            upRightLiftServo.setPower(manualPower);
            downRightLiftServo.setPower(manualPower);
        }
    }
}