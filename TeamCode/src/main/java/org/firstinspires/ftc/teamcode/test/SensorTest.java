/*package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Sensor_Test", group = "Test")
public class SensorTest extends LinearOpMode {

    private ColorSensor intakeSensorV2;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeSensorV2 = hardwareMap.get(ColorSensor.class, "senzor_intake");

        telemetry.addData("Status: ", "init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int proximitateRaw = intakeSensorV2.alpha();

            telemetry.addLine("=== CALIBRARE SENZOR REV V2 ===");
            telemetry.addData("Valoare Proximitate Curentă", proximitateRaw);

            telemetry.update();
        }
    }
}
 */