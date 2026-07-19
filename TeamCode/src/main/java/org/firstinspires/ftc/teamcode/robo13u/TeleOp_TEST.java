package org.firstinspires.ftc.teamcode.robo13u;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.Command;
import org.firstinspires.ftc.teamcode.commandbase.InstantCommand;
import org.firstinspires.ftc.teamcode.commandbase.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandbase.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandbase.SleepCommand;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@Config
@TeleOp
public class TeleOp_TEST extends LinearOpMode {

    private static Robot robot;
    private GamepadEx gamepad;
    private Command runningCommand;
    private Follower follower;
    private final Pose resetPose = new Pose(9, 9, Math.toRadians(180));
    private final Pose basePose = new Pose(38, 33, Math.toRadians(225));
    private PathChain base;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, resetPose);
        gamepad = new GamepadEx(gamepad1);

        new ParallelCommand(
                new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.TEST)),
                new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT)),
                new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                new InstantCommand(()-> robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE)),
                new InstantCommand(()-> robot.intake.setIntakeState(Intake.IntakeState.IDLE)),
                new InstantCommand(()-> robot.outtake.setGoalXY(0,144))
        ).run(new TelemetryPacket());

        buildBasePath();

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            gamepad.update();

            if(gamepad.wasJustPressed(GamepadEx.Button.cross)){
                if(robot.intake.getIntakeMotorState()!= Intake.IntakeMotorState.INTAKING) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                            new InstantCommand(() -> robot.intake.setLockState(Intake.LockState.LOCKED))
                    );
                } else if (robot.intake.getIntakeMotorState()!= Intake.IntakeMotorState.LOCKED) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setLockState(Intake.LockState.LOCKED))
                    );
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.circle)){
                if(robot.intake.getIntakeMotorState()== Intake.IntakeMotorState.INTAKING || robot.intake.getIntakeMotorState()== Intake.IntakeMotorState.LOCKED) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.REVERSE)),
                            new InstantCommand(() -> robot.intake.setLockState(Intake.LockState.LOCKED)),
                            new SleepCommand(0.5),
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    );
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.right_bumper)) {
                if(robot.outtake.getShooterState() == Outtake.ShooterState.INIT) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.TEST)),
                            new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.TEST)),
                            new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                            new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    );
                }
                else if(robot.outtake.getShooterState() == Outtake.ShooterState.TEST && robot.intake.getLockState() == Intake.LockState.LOCKED){
                        runningCommand = new SequentialCommand(
                                new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                                new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.TEST)),
                                new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.TEST)),
                                new SleepCommand(0.05),
                                new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING))
                        );
                }
                else if(robot.outtake.getShooterState() == Outtake.ShooterState.TEST && robot.intake.getLockState() == Intake.LockState.TRANSFER) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                            new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.INIT)),
                            new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.TEST)),
                            new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    );
                }
            }
            /*
            if(gamepad.wasJustPressed(GamepadEx.Button.options)) {
                if (!follower.isBusy()) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> follower = Constants.createFollower(hardwareMap)),
                            new InstantCommand(this::buildBasePath),

                            new InstantCommand(() -> robot.outtake.setTurretState(Outtake.TurretState.BASE)),
                            new SleepCommand(0.05),

                            new InstantCommand(() -> follower.setStartingPose(resetPose)),
                            new SleepCommand(0.05),

                            new InstantCommand(() -> follower.followPath(base)),
                            new WaitUntilCommand(() -> !follower.isBusy()),
                            new InstantCommand(() -> follower.holdPoint(follower.poseTracker.getPose())),
                            new SleepCommand(0.05),

                            new InstantCommand(() -> robot.lift.setLiftState(Lift.LiftState.UP)),
                            new SleepCommand(0.5),
                            new InstantCommand(() -> robot.lift.setLiftState(Lift.LiftState.IDLE))
                    );
                }
                else {
                    runningCommand = null;
                }
            }
             */

            if(gamepad.wasJustPressed(GamepadEx.Button.options)) {
                if (robot.lift.getLiftState() == Lift.LiftState.IDLE) {
                    runningCommand = new InstantCommand(() -> robot.lift.setLiftState(Lift.LiftState.UP));
                }
                else if (robot.lift.getLiftState() == Lift.LiftState.UP) {
                    runningCommand = new InstantCommand(() -> robot.lift.setLiftState(Lift.LiftState.IDLE));
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.left_bumper)){
                if (robot.outtake.getTurretState() == Outtake.TurretState.FRONT) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO));
                } else if (robot.outtake.getTurretState() == Outtake.TurretState.AUTO) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT));
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.share)){
                if (robot.outtake.getGoalX() == 144) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setGoalXY(0,144));
                }
                else if (robot.outtake.getGoalX() == 0) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setGoalXY(144,144));
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_up)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementHoodUp());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_down)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementHoodDown());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_left)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementTurretLeft());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_right)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementTurretRight());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.square)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementShooterSlower());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.triangle)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementShooterFaster());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.ps)){
                robot.mecanumDrive.imu.setYaw(0);

                robot.poseTracker.setPose(resetPose);
                robot.poseTracker.setPose(resetPose);
                robot.poseTracker.setPose(resetPose);

                runningCommand = new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1))
                );
            }
             /*
            if(gamepad.wasJustPressed(GamepadEx.Button.ps)){
                runningCommand = new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.mecanumDrive.imu.setYaw(0)),
                        new InstantCommand(()-> robot.poseTracker.setPose(resetPose)),
                        new InstantCommand(()-> robot.poseTracker.setPose(resetPose)),
                        new InstantCommand(()-> robot.poseTracker.setPose(resetPose)),
                        new SleepCommand(0.05),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1))
                );
            }
             */

            if (robot.outtake.getGoalX() == 0) {
                robot.drive(-gamepad.gamepad.left_stick_y,
                        gamepad.gamepad.left_stick_x,
                        (gamepad.gamepad.left_trigger - gamepad.gamepad.right_trigger));
            }
            else {
                robot.drive(gamepad.gamepad.left_stick_y,
                        -gamepad.gamepad.left_stick_x,
                        (gamepad.gamepad.left_trigger - gamepad.gamepad.right_trigger));
            }

            if (runningCommand != null) {
                if (runningCommand.run(new TelemetryPacket())) {
                    runningCommand = null;
                }
            }

            telemetry.addData("distance: ", robot.outtake.getTrueShooterDistance(robot.poseTracker.getPose()));
            telemetry.addData("shooterTPS: ", robot.outtake.leftShooterMotor.getVelocity());
            telemetry.addData("shooterMultiplier: ", robot.outtake.getShooterMultiplier());
            telemetry.addData("hoodPosition: ", robot.outtake.hoodServo.getPosition());
            telemetry.addData("hoodMultiplier: ", robot.outtake.getHoodMultiplier());

            telemetry.update();

            robot.update();
        }

    }

    public void buildBasePath() {
        base = follower.pathBuilder()
                .addPath(new BezierLine(resetPose, basePose))
                .setLinearHeadingInterpolation(resetPose.getHeading(), basePose.getHeading(), 0.75)
                .build();
    }

}
