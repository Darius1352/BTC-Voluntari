package org.firstinspires.ftc.teamcode.robo13u.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Motor;

@Config
public class Intake {
    public DcMotorEx leftIntakeMotor;
    public DcMotorEx rightIntakeMotor;
    public DistanceSensor leftSensor;
    public DigitalChannel rightSensor;
    public Servo leftLED;
    public Servo rightLED;
    public IntakeState intakeState;
    public static IntakeMotorState intakeMotorState;
    public static double COLOR_BALL = 0.5;
    public static double COLOR_EMPTY = 0;
    private boolean cachedLeftBall = false;
    private final ElapsedTime distanceTimer = new ElapsedTime();

    public enum IntakeState{
        INTAKE,
        TRANSFER,
        TRANSFER_SLOW,
        REVERSE,
        IDLE;
    }

    public enum IntakeMotorState{
        INTAKING(1),
        INTAKING_SLOW(0.5),
        LOCKING(0.8),
        LOCKED(0),
        REVERSE(-1);

        double power;
        IntakeMotorState(double power){
            this.power = power;
        }
        double getPower(){
            return this.power;
        }
    }

    public Intake(DcMotorEx leftIntakeMotor, DcMotorEx rightIntakeMotor, Servo leftLED, Servo rightLED, DistanceSensor leftSensor, DigitalChannel rightSensor){
        this.leftIntakeMotor = leftIntakeMotor;
        this.rightIntakeMotor = rightIntakeMotor;
        this.leftLED = leftLED;
        this.rightLED = rightLED;
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;

        leftLED.setPosition(COLOR_BALL);
        rightLED.setPosition(COLOR_BALL);

        rightSensor.setMode(DigitalChannel.Mode.INPUT);
        distanceTimer.reset();

        setIntakeMotorState(IntakeMotorState.LOCKED);
        setIntakeState(IntakeState.IDLE);

        this.leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setIntakeState(IntakeState newIntakeState) {
        this.intakeState = newIntakeState;
    }
    public IntakeState getIntakeState(){
        return intakeState;
    }

    public void setIntakeMotorState(IntakeMotorState newState) {
        intakeMotorState = newState;
    }
    public IntakeMotorState getIntakeMotorState() {
        return intakeMotorState;
    }

    public boolean hasLeftBall() {
        if (distanceTimer.seconds() >= 0.05) {
            double distance = leftSensor.getDistance(DistanceUnit.MM);
            cachedLeftBall = !Double.isNaN(distance) && distance <= 20.0;
            distanceTimer.reset();
        }
        return cachedLeftBall;
    }

    public boolean hasRightBall() {
        return !rightSensor.getState();
    }

    public void update(double voltage){
        double intakePower = intakeMotorState.getPower() * (12.0 / voltage);
        leftIntakeMotor.setPower(intakePower);
        rightIntakeMotor.setPower(intakePower);
        /*
        leftLED.setPosition(hasLeftBall() ? COLOR_BALL : COLOR_EMPTY);
        rightLED.setPosition(hasRightBall() ? COLOR_BALL : COLOR_EMPTY);
         */
    }

}