/*package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utils.GamepadEx;

public class shooterTest extends LinearOpMode {

    public DcMotorEx leftShooterMotor;
    public DcMotorEx rightShooterMotor;

    public PIDFController shooterPIDF;
    public SimpleMotorFeedforward feedforward;

    public static double skStatic = 0, skVelocity = 0, skAcceleration = 0;
    public static double sP = 0, sI = 0, sD = 0;
    public static double test_TPS = 2000;
    public static double shooter_multiplier = 1;
    public static double targetTPS = 0;
    public static ShooterState shooterState;

    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    public GamepadEx gamepadEx;

    public enum ShooterState {
        IDLE(0),
        TEST(0);

        double power;

        ShooterState (double power) {
            this.power = power;
        }

        double getPower() {
            return this.power;
        }
    }

    public ShooterState getShooterState() {
        return shooterState;
    }
    public void setShooterState (ShooterState newState) {
        shooterState = newState;
    }
    private double calculateTestTargetTPS() {
        return (test_TPS * shooter_multiplier);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");

        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterPIDF = new PIDFController(sP, sI, sD, 0);
        feedforward = new SimpleMotorFeedforward(skStatic, skVelocity, skAcceleration);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        voltage = voltageSensor.getVoltage();

        gamepadEx = new GamepadEx(gamepad1);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            gamepadEx.update();

            if (gamepadEx.wasJustPressed(GamepadEx.Button.right_bumper)) {
                if (getShooterState() == ShooterState.IDLE) {
                    setShooterState(ShooterState.TEST);
                }
                else if (getShooterState() == ShooterState.TEST) {
                    setShooterState(ShooterState.IDLE);

                }
            }

            if(getShooterState() == ShooterState.TEST){
                targetTPS = calculateTestTargetTPS();

                shooterPIDF.setPIDF(sP, sI, sD, 0);
                feedforward = new SimpleMotorFeedforward(skStatic, skVelocity, skAcceleration);

                double currentTPS = rightShooterMotor.getVelocity();

                double pidCorrection = shooterPIDF.calculate(currentTPS, targetTPS);
                double ffOutput = feedforward.calculate(targetTPS) * (12.0 / voltage);

                double finalPower = Math.max(0.0, Math.min(1.0, pidCorrection + ffOutput));

                leftShooterMotor.setPower(finalPower);
                rightShooterMotor.setPower(finalPower);
            }
            else if(getShooterState() == ShooterState.IDLE) {
                targetTPS = 0;

                shooterPIDF.setPIDF(sP, sI, sD, 0);
                feedforward = new SimpleMotorFeedforward(skStatic, skVelocity, skAcceleration);

                double currentTPS = rightShooterMotor.getVelocity();

                double pidCorrection = shooterPIDF.calculate(currentTPS, targetTPS);
                double ffOutput = feedforward.calculate(targetTPS) * (12.0 / voltage);

                double finalPower = Math.max(0.0, Math.min(1.0, pidCorrection + ffOutput));

                leftShooterMotor.setPower(finalPower);
                rightShooterMotor.setPower(finalPower);
            }

        }

    }

}
 */
