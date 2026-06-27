package org.firstinspires.ftc.teamcode.robo13u;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Outtake;

import java.util.List;

public class Robot {

    public static boolean DEBUGGING_TELEMETRY = false;

    public final DcMotorEx rightIntakeMotor;
    public final DcMotorEx leftIntakeMotor;

    public final DcMotorEx leftShooterMotor;
    public final DcMotorEx rightShooterMotor;

    public final Servo lockServo;
    public final Servo hoodServo;

    public final Servo leftTurretServo;
    public final Servo rightTurretServo;

    public final CRServo upLeftLiftServo;
    public final CRServo downLeftLiftServo;
    public final CRServo upRightLiftServo;
    public final CRServo downRightLiftServo;

    public final MecanumDrive mecanumDrive;
    public final Intake intake;
    public final Outtake outtake;
    public final Lift lift;

    private final VoltageSensor voltageSensor;
    public final PoseTracker poseTracker;

    private final MultipleTelemetry telemetry;
    private final LinearOpMode linearOpMode;

    private final LynxModule controlHub;
    private final LynxModule expansionHub;

    public static double lastVoltageReading = 12.0;
    public HardwareMap hardwareMap = null;

    public Robot (LinearOpMode linearOpMode, Pose startPose) {

        hardwareMap = linearOpMode.hardwareMap;

        rightIntakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");

        lockServo = hardwareMap.get(Servo.class, "lockServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        leftTurretServo = hardwareMap.get(Servo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(Servo.class, "rightTurretServo");

        upLeftLiftServo = hardwareMap.get(CRServo.class, "upLeftLiftServo");
        downLeftLiftServo = hardwareMap.get(CRServo.class, "downLeftLiftServo");
        upRightLiftServo = hardwareMap.get(CRServo.class, "upRightLiftServo");
        downRightLiftServo = hardwareMap.get(CRServo.class, "downRightLiftServo");

        poseTracker = new PoseTracker(new PinpointLocalizer(hardwareMap, Constants.localizerConstants));

        if(startPose != null){
            poseTracker.setStartingPose(startPose);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        rightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lockServo.setDirection(Servo.Direction.FORWARD);
        hoodServo.setDirection(Servo.Direction.FORWARD);

        leftTurretServo.setDirection(Servo.Direction.FORWARD);
        rightTurretServo.setDirection(Servo.Direction.FORWARD);

        upLeftLiftServo.setDirection(CRServo.Direction.FORWARD);
        downLeftLiftServo.setDirection(CRServo.Direction.FORWARD);
        upRightLiftServo.setDirection(CRServo.Direction.FORWARD);
        downRightLiftServo.setDirection(CRServo.Direction.FORWARD);

        mecanumDrive = new MecanumDrive(linearOpMode, this);
        intake = new Intake(rightIntakeMotor, leftIntakeMotor, lockServo);
        outtake = new Outtake(leftShooterMotor, rightShooterMotor, hoodServo, leftTurretServo, rightTurretServo);
        lift = new Lift(upLeftLiftServo, downLeftLiftServo, upRightLiftServo, downRightLiftServo);

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        this.linearOpMode = linearOpMode;
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
        TelemetryPacket packet = new TelemetryPacket();

        controlHub.clearBulkCache();
        expansionHub.clearBulkCache();

        lastVoltageReading = voltageSensor.getVoltage();

        poseTracker.update();

        intake.update(lastVoltageReading);
        outtake.update(lastVoltageReading, poseTracker.getPose(), poseTracker.getVelocity(), poseTracker.getAcceleration(), packet);
        /*
        double currentSeconds = System.nanoTime() / 1e9;
        double loopTime = currentSeconds - lastTelemetryLooptimeLog;

        packet.put("Looptime ms: ", String.format("%.2f ms", loopTime * 1000));
        packet.put("Looptime hz: ", String.format("%.2f hz", 1.0 / loopTime));

        packet.put("Robot Pose: ", poseTracker.getPose().toString());
        packet.put("Robot Velocity: ", poseTracker.getVelocity().toString());
        packet.put("Robot Acceleretion: ", poseTracker.getAcceleration().toString());

        packet.put("shooter RPM: ", (outtake.rightShooterMotor.getVelocity()) / 28.0 * 60.0);
        packet.put("shooter TPS: ", outtake.rightShooterMotor.getVelocity());
        packet.put("shooter power: ", outtake.rightShooterMotor.getPower());
        packet.put("TARGET TPS: ", outtake.getTargetTPS());
        packet.put("TARGET RPM: ", (outtake.getTargetTPS()) / 28.0 * 60.0);

        packet.put("DistanceToGoal: ", outtake.getTrueShooterDistance(poseTracker.getPose()));

        packet.put("IntakeMotorState: ", intake.getIntakeMotorState().toString());
        packet.put("LockState: ", intake.getLockState().toString());
        packet.put("TurretState: ", outtake.getTurretState().toString());
        packet.put("ShooterState: ", outtake.getShooterState().toString());
        packet.put("HoodState: ", outtake.getHoodState().toString());
        packet.put("LiftState: ", lift.getLiftState().toString());

        packet.put("IntakeState: ", intake.getIntakeState().toString());
        packet.put("OuttakeState: ", outtake.getOuttakeState().toString());

        lastTelemetryLooptimeLog = currentSeconds;

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        */
    }

    public void drive(double forward, double strafe, double rotate)
    {
        mecanumDrive.driveRobotCentric(forward, strafe, rotate);
    }

}
