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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Outtake;

@Autonomous
public class AUTO_BLUE extends LinearOpMode {
    private static Robot robot;
    private Follower follower;
    private ElapsedTime pathTimer;
    private final Pose startPose = new Pose(40, 9, Math.toRadians(90));
    private final Pose bottomCollectPose = new Pose(24, 33, Math.toRadians(90));
    private final double waitBottomC = 2.5;
    private final Pose bottomShootPose = new Pose(45, 12, Math.toRadians(180));
    private final double waitBottomS = 1.5;
    private final Pose firstCollectPose = new Pose(10, 11, Math.toRadians(180));
    private final double waitFirstC = 1.75;
    private final Pose firstShootPose = new Pose(45, 12, Math.toRadians(180));
    private final double waitFirstS = 1.5;
    private final Pose secondCollectPose = new Pose(13, 42, Math.toRadians(138));
    private final double waitSecondC = 2;
    private final Pose slide1Pose = new Pose(10, 50, Math.toRadians(90));
    private final Pose secondShootPose = new Pose(47.5, 12, Math.toRadians(135));
    private final double waitSecondS = 1.5;
    private final Pose thirdCollectPose = new Pose(10, 11, Math.toRadians(180));
    private final double waitThirdC = 1.75;
    private final Pose thirdShootPose = new Pose(47.5, 12, Math.toRadians(180));
    private final double waitThirdS = 1.5;
    private final Pose fourthCollectPose = new Pose(13, 35, Math.toRadians(147));
    private final double waitFourthC = 3.5;
    private final Pose slide2Pose = new Pose(10, 55, Math.toRadians(90));
    private final Pose fourthShootPose = new Pose(45, 12, Math.toRadians(115));
    private final double waitFourthS = 1.5;
    private final Pose fifthCollectPose = new Pose(10, 11, Math.toRadians(180));
    private final double waitFifthC = 2;
    private final Pose fifthShootPose = new Pose(45, 12, Math.toRadians(180));
    private final double waitFifthS = 1.5;
    private final Pose parkPose = new Pose(11, 11, Math.toRadians(180));
    private final double waitPark = 2;

    private PathChain bottomCollect, bottomShoot, firstCollect, firstShoot, secondCollect, slide1, secondShoot, thirdCollect, thirdShoot, fourthCollect, slide2, fourthShoot, fifthCollect, fifthShoot, park;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.mecanumDrive.imu.resetYaw();
        robot.poseTracker.setCurrentPoseWithOffset(startPose);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        
        pathTimer = new ElapsedTime();

        buildPaths();

        new InstantCommand(()-> robot.outtake.setGoalXY(0, 144));
        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT));
        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR));
        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT));
        new InstantCommand(()-> robot.outtake.setPadOffset(2));
        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1));
        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1));
        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED));
        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED));

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("robot", "init");

            new SequentialCommand(
                    new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                    new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                    new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                    new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT)),
                    new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                    new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    ).run(new TelemetryPacket());
        }

        Command mainCommand = new SequentialCommand(
                //INIT
                new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                        ),

                //PRELOAD
                new SequentialCommand(
                        new InstantCommand(()->robot.outtake.setGoalXY(0,144)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.9)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new SleepCommand(0.8),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.8),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT))
                ),

                //BOTTOM SPIKE
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(bottomCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitBottomC),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(bottomShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.9)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitBottomS),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.95),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT))
                ),

                //FIRST
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(firstCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitFirstC),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(firstShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.9)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitFirstS),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.95),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT))
                ),

                //SECOND
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(secondCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.85),
                        new InstantCommand(()-> follower.followPath(slide1)),
                        new WaitUntilCommand(()-> !follower.isBusy()|| pathTimer.seconds() > waitSecondC + 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(secondShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.9)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitSecondS),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.95),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT))
                ),

                //THIRD
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(thirdCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitThirdC),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(thirdShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(0.98)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.95)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitThirdS),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.95),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT))
                ),
                //FOURTH
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(fourthCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.85),
                        new InstantCommand(()-> follower.followPath(slide2)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitFourthC + 0.5),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(fourthShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.9)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitFourthS),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.95),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT))
                ),
                //FIFTH
                new SequentialCommand(
                        //COLLECTING
                        new InstantCommand(()-> follower.followPath(fifthCollect)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.45),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitFifthC),
                    new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        //SHOOTING
                        new InstantCommand(()-> follower.followPath(fifthShoot)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(0.98)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                        new WaitUntilCommand(()-> follower.getCurrentTValue() > 0.67),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKING)),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitFifthS),
                        new InstantCommand(()-> follower.holdPoint(follower.getPose())),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                        new SleepCommand(0.95),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT))
                ),

                //PARK
                new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                        new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT)),
                        new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                        new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                        new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                        new InstantCommand(()-> follower.followPath(park)),
                        new InstantCommand(()-> pathTimer.reset()),
                        new WaitUntilCommand(()-> !follower.isBusy() || pathTimer.seconds() > waitPark),
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
            robot.update();
        }

    }

    public void buildPaths() {
        bottomCollect = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(23.5, 17), new Pose(23.5, 17), bottomCollectPose))
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
                .setLinearHeadingInterpolation(firstCollectPose.getHeading(), secondCollectPose.getHeading(), 0.75)
                .build();
        slide1 = follower.pathBuilder()
                .addPath(new BezierLine(secondCollectPose, slide1Pose))
                .setLinearHeadingInterpolation(secondCollectPose.getHeading(), slide1Pose.getHeading(), 0.75)
                .build();
        secondShoot = follower.pathBuilder()
                .addPath(new BezierLine(slide1Pose, secondShootPose))
                .setLinearHeadingInterpolation(slide1Pose.getHeading(), secondShootPose.getHeading(), 0.25)
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
        slide2 = follower.pathBuilder()
                .addPath(new BezierLine(fourthCollectPose, slide2Pose))
                .setLinearHeadingInterpolation(fourthCollectPose.getHeading(), slide2Pose.getHeading(), 0.75)
                .build();
        fourthShoot = follower.pathBuilder()
                .addPath(new BezierLine(slide2Pose, fourthShootPose))
                .setLinearHeadingInterpolation(slide2Pose.getHeading(), fourthShootPose.getHeading(), 0.25)
                .build();
        fifthCollect = follower.pathBuilder()
                .addPath(new BezierLine(fourthShootPose, fifthCollectPose))
                .setTangentHeadingInterpolation()
                .build();
        fifthShoot = follower.pathBuilder()
                .addPath(new BezierLine(fifthCollectPose, fifthShootPose))
                .setTangentHeadingInterpolation()
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(fifthShootPose, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();

    }

}
