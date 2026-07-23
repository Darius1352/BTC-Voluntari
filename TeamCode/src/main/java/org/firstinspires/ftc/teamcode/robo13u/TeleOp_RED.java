package org.firstinspires.ftc.teamcode.robo13u;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.Command;
import org.firstinspires.ftc.teamcode.commandbase.InstantCommand;
import org.firstinspires.ftc.teamcode.commandbase.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandbase.SleepCommand;
import org.firstinspires.ftc.teamcode.commandbase.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robo13u.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;

@Config
@TeleOp
public class TeleOp_RED extends LinearOpMode {

    private static Robot robot;
    private GamepadEx gamepad;
    private Command runningCommand;
    private final Pose resetPose = new Pose(9, 9, Math.toRadians(0));
    private final Pose basePose = new Pose(38, 33, Math.toRadians(90)); //225 - heading
    private PathChain base;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, resetPose);
        gamepad = new GamepadEx(gamepad1);

        new SequentialCommand(
                new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.PRESHOOT)),
                new InstantCommand(()-> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                new InstantCommand(()-> robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE)),
                new InstantCommand(()-> robot.intake.setIntakeState(Intake.IntakeState.IDLE)),
                new InstantCommand(()-> robot.outtake.setShooterMultiplier(1.1)),
                new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.8)),
                new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                new InstantCommand(()-> robot.outtake.setGoalXY(144,144))
        ).run(new TelemetryPacket());

        buildBasePath();

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            gamepad.update();

            if(gamepad.wasJustPressed(GamepadEx.Button.cross)){
                if(robot.intake.getIntakeMotorState()!= Intake.IntakeMotorState.INTAKING) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                            new InstantCommand(() -> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.INTAKE))
                    );
                } else if (robot.intake.getIntakeMotorState()!= Intake.IntakeMotorState.LOCKED) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                            new InstantCommand(() -> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.IDLE))
                    );
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.circle)){
                if(robot.intake.getIntakeMotorState()== Intake.IntakeMotorState.INTAKING || robot.intake.getIntakeMotorState()== Intake.IntakeMotorState.LOCKED) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.REVERSE)),
                            new InstantCommand(() -> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.REVERSE)),
                            new SleepCommand(0.5),
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.IDLE))
                    );
                }
            }

            /*
            if(gamepad.wasJustPressed(GamepadEx.Button.right_bumper)) {
                if(robot.outtake.getShooterState() == Outtake.ShooterState.PRESHOOT) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                            new InstantCommand(() -> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                            new InstantCommand(() -> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.IDLE)),
                            new InstantCommand(() -> robot.outtake.setOuttakeState(Outtake.OuttakeState.SHOOT)),
                            new SleepCommand(0.3),
                            new InstantCommand(() -> robot.outtake.setLockState(Outtake.LockState.TRANSFER)),
                            new SleepCommand(0.1),
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.INTAKE)),
                            new SleepCommand(1),
                            new InstantCommand(() -> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                            new InstantCommand(() -> robot.outtake.setShooterState(Outtake.ShooterState.PRESHOOT)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.IDLE)),
                            new InstantCommand(() -> robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE))
                    );
                }
                else if(robot.outtake.getShooterState() == Outtake.ShooterState.SHOOT) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(() -> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(() -> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                            new InstantCommand(() -> robot.outtake.setShooterState(Outtake.ShooterState.PRESHOOT)),
                            new InstantCommand(() -> robot.intake.setIntakeState(Intake.IntakeState.IDLE)),
                            new InstantCommand(() -> robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE))
                    );
                }

            }
            */

            if(gamepad.wasJustPressed(GamepadEx.Button.right_bumper)) {
                if(robot.outtake.getShooterState() == Outtake.ShooterState.PRESHOOT) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                            new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                            new InstantCommand(()-> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    );
                }
                else if(robot.outtake.getShooterState() == Outtake.ShooterState.SHOOT && robot.outtake.getLockState() == Outtake.LockState.LOCKED){
                    runningCommand = new SequentialCommand(
                            new InstantCommand(()-> robot.outtake.setLockState(Outtake.LockState.TRANSFER)),
                            new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.SHOOT)),
                            new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.AUTO)),
                            new SleepCommand(0.25),
                            new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING))
                    );
                }
                else if(robot.outtake.getShooterState() == Outtake.ShooterState.SHOOT && robot.outtake.getLockState() == Outtake.LockState.TRANSFER) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(()-> robot.outtake.setLockState(Outtake.LockState.LOCKED)),
                            new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.PRESHOOT)),
                            new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                            new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    );
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.options)) {
                runningCommand = new SequentialCommand(
                        new InstantCommand(() -> robot.outtake.setTurretState(Outtake.TurretState.BASE)),
                        new SleepCommand(0.05),

                        new InstantCommand(() -> robot.follower.setStartingPose(resetPose)),
                        new SleepCommand(0.05),

                        new InstantCommand(() -> robot.follower.followPath(base)),
                        new WaitUntilCommand(() -> !robot.follower.isBusy()),
                        new InstantCommand(() -> robot.follower.holdPoint(robot.follower.poseTracker.getPose())),
                        new SleepCommand(0.05),
                        new InstantCommand(() -> robot.lift.setLiftState(Lift.LiftState.UP)),
                        new SleepCommand(0.5),
                        new InstantCommand(() -> robot.lift.setLiftState(Lift.LiftState.IDLE))
                );
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.left_bumper)){
                if (robot.outtake.getTurretState() == Outtake.TurretState.FRONT) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO));
                } else if (robot.outtake.getTurretState() == Outtake.TurretState.AUTO) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT));
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_up)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementHoodUp());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_down)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementHoodDown());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_right)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementTurretRight());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_left)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementTurretLeft());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.square)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementShooterSlower());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.triangle)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementShooterFaster());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.ps)){
                robot.mecanumDrive.imu.setYaw(0);

                robot.follower.setPose(resetPose);
                robot.follower.setPose(resetPose);
                robot.follower.setPose(resetPose);

                runningCommand = new SequentialCommand(
                        new InstantCommand(()-> robot.outtake.setPadOffset(2)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1.1)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(0.8))
                );
            }

            robot.drive(-gamepad.gamepad.left_stick_y,
                    gamepad.gamepad.left_stick_x,
                    (gamepad.gamepad.left_trigger - gamepad.gamepad.right_trigger));

            if (runningCommand != null) {
                if (runningCommand.run(new TelemetryPacket())) {
                    runningCommand = null;
                }
            }

            robot.update();

        }

    }

    private void buildBasePath() {
        base = robot.follower.pathBuilder()
                .addPath(new BezierLine(resetPose, basePose))
                .setLinearHeadingInterpolation(resetPose.getHeading(), basePose.getHeading(), 0.75)
                .build();
    }

}
