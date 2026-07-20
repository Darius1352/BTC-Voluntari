package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CRServoWrapper implements CRServo {
    CRServo servo;
    private double lastPower = -1e7;
    private double TOLERANCE = 0.05;

    public CRServoWrapper(CRServo servo) {
        this.servo = servo;
    }

    public void setTolerance(double tolerance) {
        this.TOLERANCE = tolerance;
    }

    @Override
    public void setPower(double power) {
        if (Math.abs(power - lastPower) > TOLERANCE || (power == 0.0 && lastPower != 0.0)) {
            servo.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public double getPower() {
        return lastPower;
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
        lastPower = -1e7;
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }
}