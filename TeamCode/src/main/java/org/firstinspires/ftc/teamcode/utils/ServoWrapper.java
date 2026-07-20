package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class ServoWrapper implements Servo {
    Servo servo;
    private double lastPosition = -1e7;
    private double TOLERANCE = 0.003;

    public ServoWrapper(Servo servo) {
        this.servo = servo;
    }

    public void setTolerance(double tolerance) {
        this.TOLERANCE = tolerance;
    }

    @Override
    public void setPosition(double position) {
        if (Math.abs(position - lastPosition) > TOLERANCE) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    @Override
    public double getPosition() {
        return lastPosition;
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
        lastPosition = -1e7;
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
        lastPosition = -1e7;
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