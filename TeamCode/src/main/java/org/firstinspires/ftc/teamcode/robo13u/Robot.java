package org.firstinspires.ftc.teamcode.robo13u;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utils.CRServoWrapper;
import org.firstinspires.ftc.teamcode.utils.Motor;
import org.firstinspires.ftc.teamcode.utils.ServoWrapper;

import java.util.List;

@Config
public class Robot {
    public static boolean DEBUGGING_TELEMETRY = false;

    public final DcMotorEx leftIntakeMotor;
    public final DcMotorEx rightIntakeMotor;

    public final DcMotorEx leftShooterMotor;
    public final DcMotorEx rightShooterMotor;

    public final Servo lockServo;
    public final Servo hoodServo;

    public final Servo leftTurretServo;
    public final Servo rightTurretServo;

    public final Servo leftLED;
    public final Servo rightLED;

    public final DigitalChannel leftSensor;
    public final DigitalChannel rightSensor;

    public final CRServo upLeftLiftServo;
    public final CRServo downLeftLiftServo;
    public final CRServo upRightLiftServo;
    public final CRServo downRightLiftServo;

    public final MecanumDrive mecanumDrive;
    public final Intake intake;
    public final Outtake outtake;
    public final Lift lift;

    private final VoltageSensor voltageSensor;

    public final Follower follower;

    private final MultipleTelemetry telemetry;
    private final LinearOpMode linearOpMode;

    private final List<LynxModule> allHubs;

    private double lastTelemetryLooptimeLog;

    public static double lastVoltageReading = 12.0;
    public HardwareMap hardwareMap = null;

    private long lastVoltageCheckTime = 0;
    private static final double VOLTAGE_TIMEOUT_MS = 250;

    public Robot (LinearOpMode linearOpMode, Pose startPose) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;

        leftIntakeMotor = new Motor(hardwareMap.get(DcMotorEx.class, "leftIntakeMotor"));
        rightIntakeMotor = new Motor(hardwareMap.get(DcMotorEx.class, "rightIntakeMotor"));

        leftShooterMotor = new Motor(hardwareMap.get(DcMotorEx.class, "leftShooterMotor"));
        rightShooterMotor = new Motor(hardwareMap.get(DcMotorEx.class, "rightShooterMotor"));

        lockServo = new ServoWrapper(hardwareMap.get(Servo.class, "lockServo"));
        hoodServo = new ServoWrapper(hardwareMap.get(Servo.class, "hoodServo"));

        leftTurretServo = new ServoWrapper(hardwareMap.get(Servo.class, "leftTurretServo"));
        rightTurretServo = new ServoWrapper(hardwareMap.get(Servo.class, "rightTurretServo"));

        leftLED = new ServoWrapper(hardwareMap.get(Servo.class, "leftLED"));
        rightLED = new ServoWrapper(hardwareMap.get(Servo.class, "rightLED"));

        leftSensor = hardwareMap.get(DigitalChannel.class, "leftSensor");
        rightSensor = hardwareMap.get(DigitalChannel.class, "rightSensor");

        upLeftLiftServo = new CRServoWrapper(hardwareMap.get(CRServo.class, "upLeftLiftServo"));
        downLeftLiftServo = new CRServoWrapper(hardwareMap.get(CRServo.class, "downLeftLiftServo"));
        upRightLiftServo = new CRServoWrapper(hardwareMap.get(CRServo.class, "upRightLiftServo"));
        downRightLiftServo = new CRServoWrapper(hardwareMap.get(CRServo.class, "downRightLiftServo"));

        follower = Constants.createFollower(hardwareMap);

        if(startPose != null){
            follower.setStartingPose(startPose);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        lastVoltageReading = voltageSensor.getVoltage();

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lockServo.setDirection(Servo.Direction.FORWARD);
        hoodServo.setDirection(Servo.Direction.FORWARD);

        leftTurretServo.setDirection(Servo.Direction.FORWARD);
        rightTurretServo.setDirection(Servo.Direction.FORWARD);

        upLeftLiftServo.setDirection(CRServo.Direction.FORWARD);
        downLeftLiftServo.setDirection(CRServo.Direction.REVERSE);
        upRightLiftServo.setDirection(CRServo.Direction.REVERSE);
        downRightLiftServo.setDirection(CRServo.Direction.FORWARD);

        mecanumDrive = new MecanumDrive(linearOpMode, this);
        intake = new Intake(leftIntakeMotor, rightIntakeMotor, lockServo, leftLED, rightLED, leftSensor, rightSensor);
        outtake = new Outtake(leftShooterMotor, rightShooterMotor, hoodServo, leftTurretServo, rightTurretServo);
        lift = new Lift(upLeftLiftServo, downLeftLiftServo, upRightLiftServo, downRightLiftServo);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if(DEBUGGING_TELEMETRY) {
            this.telemetry = new MultipleTelemetry(linearOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        else {
            this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        }
    }

    public Robot(LinearOpMode linearOpMode){
        this(linearOpMode, new Pose(0, 0, 0));
    }

    public void update() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastVoltageCheckTime > VOLTAGE_TIMEOUT_MS) {
            lastVoltageReading = voltageSensor.getVoltage();
            lastVoltageCheckTime = currentTime;
        }

        intake.update(lastVoltageReading);

        follower.update();

        Pose currentPose = follower.getPose();
        Vector currentVelocity = follower.getVelocity();
        Vector currentAcceleration = follower.getAcceleration();

        TelemetryPacket packet = new TelemetryPacket();
        packet.clearLines();

        outtake.update(lastVoltageReading, currentPose, currentVelocity, currentAcceleration, packet);

        double currentSeconds = System.nanoTime() / 1e9;
        double loopTime = currentSeconds - lastTelemetryLooptimeLog;
        lastTelemetryLooptimeLog = currentSeconds;

        if (DEBUGGING_TELEMETRY) {
            packet.put("Looptime ms: ", String.format("%.2f ms", loopTime * 1000));
            //packet.put("Looptime hz: ", String.format("%.2f hz", 1.0 / loopTime));
            /*
            packet.put("Robot Pose: ", follower.getPose().toString());
            packet.put("Robot Velocity: ", follower.getVelocity().toString());
            packet.put("Robot Acceleretion: ", follower.getAcceleration().toString());
            packet.put("DistanceToGoal: ", outtake.getTrueShooterDistance(currentPose));
            packet.put("shooter RPM: ", (outtake.leftShooterMotor.getVelocity()) / 28.0 * 60.0);
            packet.put("ShooterMultiplier: ", outtake.getShooterMultiplier());
            */
            packet.put("shooter TPS: ", outtake.leftShooterMotor.getVelocity());
            packet.put("TARGET TPS: ", outtake.getTargetTPS());
            /*
            packet.put("shooter power: ", outtake.leftShooterMotor.getPower());
            packet.put("TARGET RPM: ", (outtake.getTargetTPS()) / 28.0 * 60.0);
            packet.put("HoodMultiplier: ", outtake.getHoodMultiplier());
            packet.put("HoodPose: ", outtake.hoodServo.getPosition());
            packet.put("IntakeMotorState: ", intake.getIntakeMotorState().toString());
            packet.put("LockState: ", intake.getLockState().toString());
            packet.put("TurretState: ", outtake.getTurretState().toString());
            packet.put("ShooterState: ", outtake.getShooterState().toString());
            packet.put("HoodState: ", outtake.getHoodState().toString());
            packet.put("LiftState: ", lift.getLiftState().toString());
            packet.put("IntakeState: ", intake.getIntakeState().toString());
            packet.put("OuttakeState: ", outtake.getOuttakeState().toString());
            */
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }

    public void drive(double forward, double strafe, double rotate)
    {
        mecanumDrive.driveFieldCentric(forward, strafe, rotate);
    }

}
