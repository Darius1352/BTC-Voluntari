package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@TeleOp(name = "Positional_Servo_Test", group = "Test")
public class servoTest extends LinearOpMode {

    Servo lockServo;
    Servo hoodServo;

    int servoNumber = 1;
    GamepadEx gamepad;
    ElapsedTime timer;
    public double valoare;

    @Override
    public void runOpMode() throws InterruptedException {

        lockServo = hardwareMap.get(Servo.class, "lockServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        gamepad = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        String controlledServo = "lockServo";

        timer = new ElapsedTime();

        waitForStart();

        lockServo.setPosition(0.5);
        hoodServo.setPosition(0.5);

        timer.reset();

        while(!isStopRequested() && opModeIsActive()) {

            gamepad.update();

            double time = timer.seconds();

            timer.reset();

            switch (servoNumber){
                case 1:
                    valoare = lockServo.getPosition() + 0.5 * time * (gamepad1.left_trigger - gamepad1.right_trigger);
                    lockServo.setPosition(valoare);
                    break;
                case 2:
                    valoare = hoodServo.getPosition() + 0.5 * time * (gamepad1.left_trigger - gamepad1.right_trigger);
                    hoodServo.setPosition(valoare);
                    break;
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.cross) && servoNumber == 1) {
                servoNumber = 2;
                controlledServo = "hoodServo";
            }
            else if(gamepad.wasJustPressed(GamepadEx.Button.cross) && servoNumber == 2) {
                servoNumber = 1;
                controlledServo = "lockServo";
            }

            telemetry.addData("ControlledServo: ", controlledServo);
            telemetry.addData("lockServoPose: ", lockServo.getPosition());
            telemetry.addData("hoodServoPose: ", hoodServo.getPosition());
            telemetry.update();

        }

    }
}
