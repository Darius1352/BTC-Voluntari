package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor_Test", group = "Test")
public class motorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        boolean chassis = true;

        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rightRear");

        DcMotorEx i1 = hardwareMap.get(DcMotorEx.class, "spinnerMotor");
        DcMotorEx i2 = hardwareMap.get(DcMotorEx.class, "transferMotor");
        DcMotorEx s1 = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        DcMotorEx s2 = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        i1.setDirection(DcMotorSimple.Direction.FORWARD);
        i2.setDirection(DcMotorSimple.Direction.FORWARD);
        s1.setDirection(DcMotorSimple.Direction.FORWARD);
        s2.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {

            if(gamepad1.left_bumper) {
                chassis = !chassis;
            }

            telemetry.addData("ChassisMode ? :", chassis);

            if(chassis) {
                if (gamepad1.square) {
                    lf.setPower(0.5);
                    telemetry.addData("Running Motor: ", "leftFront");
                } else {
                    lf.setPower(0);
                }

                if (gamepad1.cross) {
                    lr.setPower(0.5);
                    telemetry.addData("Running Motor: ", "leftRear");
                } else {
                    lr.setPower(0);
                }

                if (gamepad1.triangle) {
                    rf.setPower(0.5);
                    telemetry.addData("Running Motor: ", "rightFront");
                } else {
                    rf.setPower(0);
                }

                if (gamepad1.circle) {
                    rr.setPower(0.5);
                    telemetry.addData("Running Motor: ", "rightRear");
                } else {
                    rr.setPower(0);
                }
            }
            else {
                if (gamepad1.cross) {
                    i1.setPower(0.5);
                    telemetry.addData("Running Motor: ", "leftIntake");
                } else {
                    i1.setPower(0);
                }

                if (gamepad1.square) {
                    i2.setPower(0.5);
                    telemetry.addData("Running Motor: ", "rightIntake");
                } else {
                    i2.setPower(0);
                }

                if (gamepad1.triangle) {
                    s1.setPower(0.5);
                    telemetry.addData("Running Motor: ", "leftShooter");
                } else {
                    s1.setPower(0);
                }

                if (gamepad1.circle) {
                    s2.setPower(0.5);
                    telemetry.addData("Running Motor: ", "rightShooter");
                } else {
                    s2.setPower(0);
                }
            }

            telemetry.update();

        }

    }
}
