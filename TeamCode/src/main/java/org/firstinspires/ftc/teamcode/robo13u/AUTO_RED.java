package org.firstinspires.ftc.teamcode.robo13u;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Command;
import org.firstinspires.ftc.teamcode.commandbase.InstantCommand;
import org.firstinspires.ftc.teamcode.commandbase.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandbase.SleepCommand;
import org.firstinspires.ftc.teamcode.commandbase.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Outtake;

@Autonomous
public class AUTO_RED extends LinearOpMode {

    private static Robot robot;
    public Follower follower;
    private ElapsedTime pathTimer;

    private final Pose startPose = new Pose(85, 9, Math.toRadians(90));
    private final Pose preloadPose = new Pose(89, 13, Math.toRadians(90));
    private final Pose bottomCollectPose = new Pose(118, 30, Math.toRadians(90));
    private final Pose bottomShootPose = new Pose(95, 10, 0);
    private final Pose firstCollectPose = new Pose(132, 10);
    private final Pose firstShootPose = new Pose(95, 10);
    private final Pose secondCollectPose = new Pose(132, 10);
    private final Pose secondShootPose = new Pose(95, 10);
    private final Pose thirdCollectPose = new Pose(132, 10);
    private final Pose thirdShootPose = new Pose(95, 10);
    private final Pose fourthCollectPose = new Pose(132, 10);
    private final Pose fourthShootPose = new Pose(95, 10);
    private final Pose parkPose = new Pose(132, 9, 0);

    private PathChain preload, bottomCollect, bottomShoot, firstCollect, firstShoot, secondCollect, secondShoot, thirdCollect, thirdShoot, fourthCollect, fourthShoot, park;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this, startPose);
        robot.mecanumDrive.imu.resetYaw();
        follower = robot.follower;

        pathTimer = new ElapsedTime();

        buildPaths();

        new InstantCommand(()-> robot.outtake.setGoalXY(0, 144));
        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT));
        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR));
        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE));
        new InstantCommand(()-> robot.outtake.setPadOffset(0));
        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1));
        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1));
        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED));
        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED));

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("robot", "init");

            new SequentialCommand(
                    new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                    new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                    new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                    new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE)),
                    new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                    new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
            ).run(new TelemetryPacket());
        }

        Command mainCommand = new SequentialCommand(
                //INIT
                new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE)),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                ),

                //PRELOAD
                new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new InstantCommand(()-> follower.followPath(preload)),
                        new InstantCommand(()-> follower.setMaxPower(1)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy()),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.5),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //BOTTOM SPIKE
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(bottomCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > 1),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(bottomShoot)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.7),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy()),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.5),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //FIRST
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(firstCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > 1),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(firstShoot)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.7),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy()),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.5),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //SECOND
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(secondCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > 1),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(secondShoot)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.7),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy()),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.5),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //THIRD
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(thirdCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > 1),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(thirdShoot)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.7),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy()),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.5),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),
                //FOURTH
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(fourthCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > 1),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(fourthShoot)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.7),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy()),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.5),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //PARK
                new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                        new InstantCommand(()-> follower.followPath(park)),
                        new WaitUntilCommand(()-> !follower.isBusy()),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose()))
                )

        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (mainCommand != null) {
                if (mainCommand.run(new TelemetryPacket())) {
                    mainCommand = null;
                }
            }
            follower.update();
            robot.update();
        }

    }

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preloadPose))
                .setConstantHeadingInterpolation(preloadPose.getHeading())
                .build();

        bottomCollect = follower.pathBuilder()
                .addPath(new BezierCurve(preloadPose, new Pose(119, 21), new Pose(119, 21), bottomCollectPose))
                .setConstantHeadingInterpolation(bottomCollectPose.getHeading())
                .build();
        bottomShoot = follower.pathBuilder()
                .addPath(new BezierLine(bottomCollectPose, bottomShootPose))
                .setLinearHeadingInterpolation(bottomCollectPose.getHeading(), bottomShootPose.getHeading(), 0.75)
                .build();
        firstCollect = follower.pathBuilder()
                .addPath(new BezierLine(bottomShootPose, firstCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        firstShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstCollectPose, firstShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        secondCollect = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, secondCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        secondShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondCollectPose, secondShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        thirdCollect = follower.pathBuilder()
                .addPath(new BezierLine(secondShootPose, thirdCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        thirdShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdCollectPose, thirdShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        fourthCollect = follower.pathBuilder()
                .addPath(new BezierLine(thirdShootPose, fourthCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        fourthShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthCollectPose, fourthShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(fourthShootPose, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();

    }

}
