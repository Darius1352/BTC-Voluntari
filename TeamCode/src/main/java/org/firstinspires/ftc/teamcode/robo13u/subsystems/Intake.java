package org.firstinspires.ftc.teamcode.robo13u.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    public DcMotorEx rightIntakeMotor;
    public DcMotorEx leftIntakeMotor;
    public Servo lockServo;

    public IntakeState intakeState;
    public static IntakeMotorState intakeMotorState;
    public static LockState lockState;

    public enum IntakeState{
        INTAKE(),
        TRANSFER(),
        TRANSFER_SLOW(),
        REVERSE,
        IDLE
    }

    public enum IntakeMotorState{
        INTAKING(1),
        INTAKING_SLOW(0.4),
        INTAKING_FAR(1),
        LOCKING(0.2),
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
        TRANSFER(0.7),
        LOCKED(0.43);

        double power;

        LockState(double Power) {
            this.power = Power;
        }

        double getPosition() {
            return this.power;
        }
    }

    public Intake(DcMotorEx rightIntakeMotor, DcMotorEx leftIntakeMotor, Servo lockServo){
        this.rightIntakeMotor = rightIntakeMotor;
        this.leftIntakeMotor = leftIntakeMotor;
        this.lockServo = lockServo;

        setLockState(LockState.LOCKED);
        setIntakeMotorState(IntakeMotorState.LOCKED);
        setIntakeState(IntakeState.IDLE);

        this.rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setIntakeMotorState(IntakeMotorState newState){
        intakeMotorState = newState;
    }

    public IntakeMotorState getIntakeMotorState(){
        return intakeMotorState;
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

    public void update(double voltage){
        double targetLockPos = lockState.getPosition();
        lockServo.setPosition(targetLockPos);

        double targetPower = intakeMotorState.getPower() * (12.0 / voltage);
        rightIntakeMotor.setPower(targetPower);
        leftIntakeMotor.setPower(targetPower);
    }

}