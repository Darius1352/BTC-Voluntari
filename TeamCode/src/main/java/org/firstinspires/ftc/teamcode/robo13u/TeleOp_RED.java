package org.firstinspires.ftc.teamcode.robo13u;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
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
public class TeleOp_RED extends LinearOpMode {

    private static Robot robot;
    private GamepadEx gamepad;
    private Command runningCommand;
    public static boolean slow_shooting = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this, new Pose(0, 0, 0));
        gamepad = new GamepadEx(gamepad1);

        new ParallelCommand(
                new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT)),
                new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE)),
                new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED)),
                new InstantCommand(()-> robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE)),
                new InstantCommand(()-> robot.intake.setIntakeState(Intake.IntakeState.IDLE)),
                new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                new InstantCommand(()-> robot.outtake.setGoalXY(0,144))
        ).run(new TelemetryPacket());

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {

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
                if(robot.outtake.getShooterState() == Outtake.ShooterState.IDLE) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.TEST)),
                            new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.TEST)),
                            new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                            new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    );
                }
                else if(robot.outtake.getShooterState() == Outtake.ShooterState.TEST && robot.intake.getLockState() == Intake.LockState.LOCKED){
                    if(!slow_shooting) {
                        runningCommand = new SequentialCommand(
                                new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                                new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.TEST)),
                                new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.TEST)),
                                new SleepCommand(0.05),
                                new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING))
                        );
                    }
                    else {
                        runningCommand = new SequentialCommand(
                                new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.TRANSFER)),
                                new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.TEST)),
                                new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.TEST)),
                                new SleepCommand(0.05),
                                new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.INTAKING_SLOW))
                        );
                    }
                }
                else if(robot.outtake.getShooterState() == Outtake.ShooterState.TEST && robot.intake.getLockState() == Intake.LockState.TRANSFER) {
                    runningCommand = new SequentialCommand(
                            new InstantCommand(()-> robot.intake.setLockState(Intake.LockState.LOCKED)),
                            new InstantCommand(()-> robot.outtake.setShooterState(Outtake.ShooterState.IDLE)),
                            new InstantCommand(()-> robot.outtake.setHoodState(Outtake.HoodState.FAR)),
                            new InstantCommand(()-> robot.intake.setIntakeMotorState(Intake.IntakeMotorState.LOCKED))
                    );
                }

            }

            if(gamepad.wasJustPressed(GamepadEx.Button.options)) {
                if (robot.lift.getLiftState() == Lift.LiftState.IDLE) {
                    runningCommand = new InstantCommand(()-> robot.lift.setLiftState(Lift.LiftState.UP));
                } else {
                    runningCommand = new InstantCommand(()-> robot.lift.setLiftState(Lift.LiftState.IDLE));
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.left_bumper)){
                if (robot.outtake.getTurretState() == Outtake.TurretState.FRONT) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.AUTO));
                } else if (robot.outtake.getTurretState() == Outtake.TurretState.AUTO) {
                    runningCommand = new InstantCommand(()-> robot.outtake.setTurretState(Outtake.TurretState.FRONT));
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.touchpad)){
                slow_shooting = !slow_shooting;
                if (slow_shooting) {
                    runningCommand = new InstantCommand(()-> robot.outtake.incrementShooterSlower());
                } else {
                    runningCommand = new InstantCommand(()-> robot.outtake.incrementShooterFaster());
                }
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_up)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementHoodUp());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_down)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementHoodDown());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_right)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementTurretLeft());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.dpad_left)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementTurretRight());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.square)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementShooterSlower());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.triangle)){
                runningCommand = new InstantCommand(()->robot.outtake.incrementShooterFaster());
            }

            if(gamepad.wasJustPressed(GamepadEx.Button.ps)){
                runningCommand = new SequentialCommand(
                        new InstantCommand(()-> robot.follower.setPose(new Pose(132, 9, 0))),
                        new InstantCommand(()-> robot.outtake.setPadOffset(0)),
                        new InstantCommand(()-> robot.outtake.setShooterMultiplier(1)),
                        new InstantCommand(()-> robot.outtake.setHoodMultiplier(1)),
                        new InstantCommand(()-> robot.mecanumDrive.imu.setYaw(0))
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

}
