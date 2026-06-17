package org.firstinspires.ftc.teamcode.robo13u.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class Lift {

    public final CRServo upLeftLiftServo;
    public final CRServo downLeftLiftServo;
    public final CRServo upRightLiftServo;
    public final CRServo downRightLiftServo;
    public static LiftState liftState;

    public enum LiftState {
        IDLE(0),
        UP(1),
        DOWN(-1);

        double power;

        LiftState (double power) {
            this.power = power;
        }

        double getPower() {
            return this.power;
        }
    }

    public Lift(CRServo upLeftLiftServo, CRServo downLeftLiftServo, CRServo upRightLiftServo, CRServo downRightLiftServo) {
        this.upLeftLiftServo = upLeftLiftServo;
        this.downLeftLiftServo = downLeftLiftServo;
        this.upRightLiftServo = upRightLiftServo;
        this.downRightLiftServo = downRightLiftServo;

        setLiftState(LiftState.IDLE);
    }

    public void setLiftState(LiftState newState) {
        liftState = newState;
        setLiftServoPower(liftState.getPower());
    }

    public LiftState getLiftState() {
        return liftState;
    }

    private void setLiftServoPower(double power) {
        upLeftLiftServo.setPower(power);
        downLeftLiftServo.setPower(power);
        upRightLiftServo.setPower(power);
        downRightLiftServo.setPower(power);
    }
}
