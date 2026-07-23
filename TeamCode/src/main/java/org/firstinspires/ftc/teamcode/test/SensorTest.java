package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor_Test", group = "Test")
public class SensorTest extends LinearOpMode {

    private DistanceSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DistanceSensor.class, "Sensor");

        telemetry.addData("Status: ", "init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double proximitateRaw = sensor.getDistance(DistanceUnit.MM);

            telemetry.addData("distanta: ", proximitateRaw);
            telemetry.update();
        }
    }
}