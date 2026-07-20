package org.firstinspires.ftc.teamcode.robo13u;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    private ElapsedTime pathTimer;
    private final Pose startPose = new Pose(101, 9, Math.toRadians(90));
    private final Pose bottomCollectPose = new Pose(119, 33, Math.toRadians(90));
    private final double waitBottomC = 2.5;
    private final Pose bottomShootPose = new Pose(96, 12, Math.toRadians(0));
    private final double waitBottomS = 1.5;
    private final Pose firstCollectPose = new Pose(132, 11, Math.toRadians(0));
    private final double waitFirstC = 1.75;
    private final Pose firstShootPose = new Pose(96, 12, Math.toRadians(0));
    private final double waitFirstS = 1.5;
    private final Pose secondCollectPose = new Pose(129, 42, Math.toRadians(42));
    private final double waitSecondC = 2;
    private final Pose slide1Pose = new Pose(132, 50, Math.toRadians(90));
    private final Pose secondShootPose = new Pose(98.5, 12, Math.toRadians(50));
    private final double waitSecondS = 1.5;
    private final Pose thirdCollectPose = new Pose(132, 11, Math.toRadians(0));
    private final double waitThirdC = 1.75;
    private final Pose thirdShootPose = new Pose(98.5, 12, Math.toRadians(0));
    private final double waitThirdS = 1.5;
    private final Pose fourthCollectPose = new Pose(129, 35, Math.toRadians(37));
    private final double waitFourthC = 3.5;
    private final Pose slide2Pose = new Pose(132, 55, Math.toRadians(90));
    private final Pose fourthShootPose = new Pose(96, 12, Math.toRadians(50));
    private final double waitFourthS = 1.5;
    private final Pose fifthCollectPose = new Pose(132, 11, Math.toRadians(0));
    private final double waitFifthC = 2;
    private final Pose fifthShootPose = new Pose(96, 12, Math.toRadians(0));
    private final double waitFifthS = 1.5;
    private final Pose parkPose = new Pose(131, 11, Math.toRadians(0));
    private final double waitPark = 2;

    private PathChain bottomCollect, bottomShoot, firstCollect, firstShoot, secondCollect, slide1, secondShoot, thirdCollect, thirdShoot, fourthCollect, slide2, fourthShoot, fifthCollect, fifthShoot, park;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, startPose);
        robot.mecanumDrive.imu.resetYaw();

        pathTimer = new ElapsedTime();

        buildPaths();

        new InstantCommand(()-> robot.outtake.setGoalXY(144, 144));
        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT));
        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR));
        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE));
        new InstantCommand(()-> robot.outtake.setPadOffset(2));
        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1));
        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1));
        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED));
        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED));

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("robot", "init");

            new SequentialCommand(
                    new InstantCommand(()->robot.outtake.setGoalXY(144,144)),
                    new InstantCommand(()-> robot.outtake.setPadOffset(2)),
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
                        new InstantCommand(()->robot.outtake.setGoalXY(144,144)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                ),

                //PRELOAD
                new SequentialCommand(
                        new InstantCommand(()->robot.outtake.setGoalXY(144,144)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(0.95)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.9)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new SleepCommand(0.75),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(1),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //BOTTOM SPIKE
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> robot.follower.followPath(bottomCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitBottomC),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> robot.follower.followPath(bottomShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.85)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitBottomS),
                        new InstantCommand(()-> robot.follower.holdPoint(robot.follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.75),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //FIRST
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> robot.follower.followPath(firstCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitFirstC),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> robot.follower.followPath(firstShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.95)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitFirstS),
                        new InstantCommand(()-> robot.follower.holdPoint(robot.follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.75),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //SECOND
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> robot.follower.followPath(secondCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.85),
                        new InstantCommand(()-> robot.follower.followPath(slide1)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy()|| pathTimer.seconds() > waitSecondC + 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> robot.follower.followPath(secondShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(0.95)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.98)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitSecondS),
                        new InstantCommand(()-> robot.follower.holdPoint(robot.follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.75),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //THIRD
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> robot.follower.followPath(thirdCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitThirdC),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> robot.follower.followPath(thirdShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(0.98)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.95)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitThirdS),
                        new InstantCommand(()-> robot.follower.holdPoint(robot.follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.75),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),
                //FOURTH
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> robot.follower.followPath(fourthCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.85),
                        new InstantCommand(()-> robot.follower.followPath(slide2)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitFourthC + 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> robot.follower.followPath(fourthShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(0.98)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitFourthS),
                        new InstantCommand(()-> robot.follower.holdPoint(robot.follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.75),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),
                //FIFTH
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> robot.follower.followPath(fifthCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitFifthC),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        //SHOOTING
                        new InstantCommand(()-> robot.follower.followPath(fifthShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(0.98)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> robot.follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitFifthS),
                        new InstantCommand(()-> robot.follower.holdPoint(robot.follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.75),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE))
                ),

                //PARK
                new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.follower.followPath(park)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> !robot.follower.isBusy() || pathTimer.seconds() > waitPark),
                        new InstantCommand(()-> robot.follower.holdPoint(robot.follower.getPose()))
                )


        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (mainCommand != null) {
                if (mainCommand.run(new TelemetryPacket())) {
                    mainCommand = null;
                }
            }
            robot.update();
        }

    }

    public void buildPaths() {
        bottomCollect = robot.follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(118.5, 17), new Pose(118.5, 17), bottomCollectPose))
                .setConstantHeadingInterpolation(bottomCollectPose.getHeading())
                .build();
        bottomShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(bottomCollectPose, bottomShootPose))
                .setLinearHeadingInterpolation(bottomCollectPose.getHeading(), bottomShootPose.getHeading(), 0.75)
                .build();
        firstCollect = robot.follower.pathBuilder()
                .addPath(new BezierLine(bottomShootPose, firstCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        firstShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(firstCollectPose, firstShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        secondCollect = robot.follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, secondCollectPose))
                .setLinearHeadingInterpolation(firstCollectPose.getHeading(), secondCollectPose.getHeading(), 0.75)
                .build();
        slide1 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(secondCollectPose, new Pose(130, 47), slide1Pose))
                .setLinearHeadingInterpolation(secondCollectPose.getHeading(), slide1Pose.getHeading(), 0.75)
                .build();
        secondShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(slide1Pose, secondShootPose))
                .setLinearHeadingInterpolation(slide1Pose.getHeading(), secondShootPose.getHeading(), 0.25)
                .build();
        thirdCollect = robot.follower.pathBuilder()
                .addPath(new BezierLine(secondShootPose, thirdCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        thirdShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(thirdCollectPose, thirdShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        fourthCollect = robot.follower.pathBuilder()
                .addPath(new BezierLine(thirdShootPose, fourthCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        slide2 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(fourthCollectPose, new Pose(130, 42), slide2Pose))
                .setLinearHeadingInterpolation(fourthCollectPose.getHeading(), slide2Pose.getHeading(), 0.75)
                .build();
        fourthShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(slide2Pose, fourthShootPose))
                .setLinearHeadingInterpolation(slide2Pose.getHeading(), fourthShootPose.getHeading(), 0.25)
                .build();
        fourthShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(fourthCollectPose, fourthShootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        fifthCollect = robot.follower.pathBuilder()
                .addPath(new BezierLine(fourthShootPose, fifthCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        fifthShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(fifthCollectPose, fifthShootPose))
                .setTangentHeadingInterpolation()
                .build();
        park = robot.follower.pathBuilder()
                .addPath(new BezierLine(fourthShootPose, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();

    }

}
