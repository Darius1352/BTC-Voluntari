package org.firstinspires.ftc.teamcode.robo13u.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Motor;

@Config
public class Intake {
    public DcMotorEx leftIntakeMotor;
    public DcMotorEx rightIntakeMotor;
    public Servo lockServo;
    public DigitalChannel leftSensor;
    public DigitalChannel rightSensor;

    public Servo leftLED;
    public Servo rightLED;

    public IntakeState intakeState;
    public static IntakeMotorState intakeMotorState;
    public static LockState lockState;
    public static double COLOR_BALL = 0.67;
    public static double COLOR_EMPTY = 0;

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

    public enum LockState {
        TRANSFER(0.8567),
        LOCKED(0.7139);

        double power;

        LockState(double Power) {
            this.power = Power;
        }

        double getPosition() {
            return this.power;
        }
    }

    public Intake(DcMotorEx leftIntakeMotor, DcMotorEx rightIntakeMotor, Servo lockServo, Servo leftLED, Servo rightLED){
        this.leftIntakeMotor = new Motor(leftIntakeMotor);
        this.rightIntakeMotor = new Motor(rightIntakeMotor);
        this.lockServo = lockServo;
        this.leftLED = leftLED;
        this.rightLED = rightLED;

        leftLED.setPosition(COLOR_BALL);
        rightLED.setPosition(COLOR_BALL);

        setLockState(LockState.LOCKED);
        setIntakeMotorState(IntakeMotorState.LOCKED);
        setIntakeState(IntakeState.IDLE);

        this.leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setLockState(LockState newState){
        lockState = newState;
    }

    public LockState getLockState(){
        return lockState;
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

    public void update(double voltage){
        lockServo.setPosition(lockState.getPosition());

        double intakePower = intakeMotorState.getPower() * (12.0 / voltage);
        leftIntakeMotor.setPower(intakePower);
        rightIntakeMotor.setPower(intakePower);
        /*
        boolean leftBall = !rightSensor.getState();
        boolean rightBall = !leftSensor.getState();

        if (leftBall) {
            leftLED.setPosition(COLOR_BALL);
        }
        else {
            leftLED.setPosition(COLOR_EMPTY);
        }

        if (rightBall) {
            rightLED.setPosition(COLOR_BALL);
        }
        else {
            rightLED.setPosition(COLOR_EMPTY);

        }
         */
    }

}