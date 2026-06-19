package org.firstinspires.ftc.teamcode.robo13u.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robo13u.Robot;
import org.firstinspires.ftc.teamcode.utils.Imu;

public class MecanumDrive {
    public static double K_STATIC = 0.15;

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;
    public Robot robot;
    public Imu imu;

    public MecanumDrive(LinearOpMode linearOpMode, Robot robot) {

        HardwareMap hardwareMap = linearOpMode.hardwareMap;

        this.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.robot = robot;
        this.imu = new Imu(linearOpMode);
    }
    public void driveFieldCentric(double forward, double strafe, double rotate)
    {
        if(rotate != 0) rotate += Math.signum(rotate) * K_STATIC;

        double heading = imu.getYaw();

        double rotX = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double rotY = forward * Math.sin(heading) + strafe * Math.cos(heading);

        driveRobotCentric(rotX, rotY, rotate);
    }
    public void driveRobotCentric(double forward, double strafe, double rotate)
    {
        strafe = strafe * 1.1;

        double denominator = Math.max(abs(forward)+abs(strafe)+abs(rotate),1);

        forward = forward / denominator;
        strafe = strafe / denominator;
        rotate = rotate / denominator;

        double leftFrontPower = - forward - strafe + rotate;
        double rightFrontPower = - forward + strafe - rotate;
        double leftRearPower = - forward + strafe + rotate;
        double rightRearPower = - forward - strafe - rotate;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }
}