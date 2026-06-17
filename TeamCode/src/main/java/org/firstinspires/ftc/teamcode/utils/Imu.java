package org.firstinspires.ftc.teamcode.utils;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Imu
{
    private IMU imu;
    private double imuPitch;
    private double lastImuYaw;
    private double YAW_OFFSET;
    private double imuYaw;
    public Imu(LinearOpMode linearOpMode)
    {
        HardwareMap hardwareMap=linearOpMode.hardwareMap;
        imu= hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }


    public double getYaw()
    {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }
    public void setYaw(double yaw){
        imu.resetYaw();
        lastImuYaw = 0;
        imuYaw = 0;
        this.YAW_OFFSET = imuYaw - yaw;
    }

    public double getPitch(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getPitch(AngleUnit.RADIANS);
    }

    public void resetYaw()
    {
        setYaw(0);
    }
}