package org.firstinspires.ftc.teamcode.robo13u.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake {

    public DcMotorEx leftShooterMotor;
    public DcMotorEx rightShooterMotor;

    public Servo hoodServo;

    public Servo leftTurretServo;
    public Servo rightTurretServo;

    public static double pad_offset = 0;
    public static double min_turret_angle = -160, max_turret_angle = 160;
    public static double test_target = 0;
    public static double GOAL_X = 144.0;
    public static double GOAL_Y = 0;
    public static double TURRET_CENTER_POS = 0.5;
    public static double TOTAL_SERVO_RANGE_DEGREES = 0;
    public static double ROBOT_CENTER_TO_TURRET_DX = 0;
    public static double ROBOT_CENTER_TO_TURRET_DY = 0;
    public static double TURRET_PIVOT_TO_BARREL = 0;
    public static double ESTIMATED_ARTIFACT_SPEED_IN_SEC = 0;
    public static double INERTIA_GAIN = 0;
    public static double ACCEL_FEEDFORWARD_GAIN = 0.05;

    public PIDFController shooterPIDF;
    public SimpleMotorFeedforward feedforward;
    public static double TPS_TO_INCHES_PER_SEC = 0;
    public static double TPS_TO_INCHES_BACKWARD = 0;
    public static double TPS_TO_INCHES_FORWARD = 0;
    public static double SHOOTER_ACCEL_GAIN = 0;
    public static double skStatic = 0, skVelocity = 0, skAcceleration = 0;
    public static double sP = 0, sI = 0, sD = 0;
    public static double test_TPS = 2000;
    public static double shooter_multiplier = 1;
    public static double targetTPS = 0;
    public static double power = 0;

    public static double min_hood_position = 0, max_hood_position = 0;
    public static double test_hood_pose = 0.5;
    public static double hood_multiplier = 1;

    public static OuttakeState outtakeState;
    public static ShooterState shooterState;
    public static HoodState hoodState;
    public static TurretState turretState;

    public double distanceToGoal = 120;
    public static boolean DEBUG_DRAWING = false;

    public enum OuttakeState {
        SHOOT(),
        IDLE(),
        UNLOCALIZED();
    }

    public enum ShooterState {
        SHOOT(1),
        IDLE(0),
        POWER(0),
        TEST(0);

        double power;

        ShooterState (double power) {
            this.power = power;
        }

        double getPower() {
            return this.power;
        }
    }

    public enum HoodState {
        AUTO(0),
        TEST(0),
        FAR(0.67),
        CLOSE(0.36);

        double position;

        HoodState (double position) {
            this.position = position;
        }

        double getPosition() {
            return this.position;
        }
    }

    public enum TurretState {
        AUTO(0),
        FRONT(0);

        double targetAngle;

        TurretState(double targetAngle) {
            this.targetAngle = targetAngle;
        }

        double getTargetAngle() {
            return this.targetAngle;
        }
    }

    public Outtake (DcMotorEx leftShooterMotor, DcMotorEx rightShooterMotor, Servo hoodServo, Servo leftTurretServo, Servo rightTurretServo) {

        this.leftShooterMotor = leftShooterMotor;
        this.rightShooterMotor = rightShooterMotor;
        this.hoodServo = hoodServo;
        this.leftTurretServo = leftTurretServo;
        this.rightTurretServo = rightTurretServo;

        setOuttakeState(OuttakeState.UNLOCALIZED);
        setHoodState(HoodState.CLOSE);
        setShooterState(ShooterState.IDLE);
        setTurretState(TurretState.FRONT);

        this.shooterPIDF = new PIDFController(sP, sI, sD, 0);

        this.leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setHoodState (HoodState newState) {
        hoodState = newState;
        if(hoodState != HoodState.AUTO && hoodState != HoodState.TEST) {
            double targetPos = hoodState.getPosition();
            hoodServo.setPosition(targetPos);
        }
    }

    public void setShooterState (ShooterState newState) {
        shooterState = newState;
    }

    public void setTurretState (TurretState newState) {
        turretState = newState;
    }

    public void setOuttakeState (OuttakeState newState) {
        this.outtakeState = newState;
    }

    public HoodState getHoodState() {
        return hoodState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public TurretState getTurretState() {
        return turretState;
    }

    public OuttakeState getOuttakeState() {
        return outtakeState;
    }

    public double getTrueShooterDistance(Pose robotPose) {

        double theta = robotPose.getHeading();

        double turretX = robotPose.getX()
                + (ROBOT_CENTER_TO_TURRET_DX * Math.cos(theta))
                - (ROBOT_CENTER_TO_TURRET_DY * Math.sin(theta));

        double turretY = robotPose.getY()
                + (ROBOT_CENTER_TO_TURRET_DX * Math.sin(theta))
                + (ROBOT_CENTER_TO_TURRET_DY * Math.cos(theta));

        double diffX = GOAL_X - turretX;
        double diffY = GOAL_Y - turretY;
        double pivotDist = Math.hypot(diffX, diffY);

        return pivotDist - TURRET_PIVOT_TO_BARREL;
    }

    private double calculateTargetTPS(double distance) {
        return shooter_multiplier * 6.7;
    }

    private double calculateTargetHood(double distance) {
        double hood_pos = hood_multiplier * 6.7;

        if (hood_pos < min_hood_position) {
            hood_pos = min_hood_position;
        }
        else if (hood_pos > max_hood_position) {
            hood_pos = max_hood_position;
        }

        return hood_pos;
    }

    private double calculateTestTargetTPS() {
        return (test_TPS * shooter_multiplier);
    }

    private double calculateTestTargetHood() {
        double hood_pos = (hood_multiplier * test_hood_pose);

        if (hood_pos < min_hood_position) {
            hood_pos = min_hood_position;
        }
        else if (hood_pos > max_hood_position) {
            hood_pos = max_hood_position;
        }

        return hood_pos;
    }

    public double angleToServoPos(double angleDegrees) {
        double positionChange = angleDegrees / TOTAL_SERVO_RANGE_DEGREES;

        double targetPos = TURRET_CENTER_POS + positionChange;

        if (targetPos > 1.0) targetPos = 1.0;
        if (targetPos < 0.0) targetPos = 0.0;

        return targetPos;
    }

    private double normalizeAngle(double degrees){
        while(degrees>180){
            degrees -= 360;
        }
        while(degrees<-180){
            degrees+=360;
        }
        return degrees;
    }

    public void incrementShooterSlower(){
        shooter_multiplier -= 0.02;
    }

    public void incrementShooterFaster(){
        shooter_multiplier += 0.02;
    }

    public void incrementHoodUp(){
        hood_multiplier += 0.025;
    }

    public void incrementHoodDown(){
        hood_multiplier -= 0.025;
    }

    public void incrementTurretLeft(){
        pad_offset+=2.5;
    }

    public void incrementTurretRight(){
        pad_offset-=2.5;
    }

    public void setShooterMultiplier(double value){
        shooter_multiplier = value;
    }

    public void setHoodMultiplier(double value){
        hood_multiplier = value;
    }

    public void setPadOffset(double value){
        pad_offset = value;
    }

    public void setTpsToInchesPerSec(double value){
        TPS_TO_INCHES_PER_SEC = value;
    }

    public void setInertiaGain(double value){
        INERTIA_GAIN = value;
    }

    public void setEstimatedArtifactSpeedInSec(double value){
        ESTIMATED_ARTIFACT_SPEED_IN_SEC = value;
    }

    public double getTargetTPS(){
        return targetTPS;
    }

    public void setTargetTPS(double value){
        test_TPS = value;
    }

    public void setPower(double value){
        power = value;
    }

    public void setGoalXY(double x, double y){
        GOAL_X = x;
        GOAL_Y = y;
    }

    public void update (double voltage, Pose robotPose, Vector robotVelocity, Vector robotAcceleration, TelemetryPacket packet) {

        double robotHeadingRadians = robotPose.getHeading();
        double robotHeadingDegrees = Math.toDegrees(robotHeadingRadians);

        double turretX = robotPose.getX()
                + (ROBOT_CENTER_TO_TURRET_DX * Math.cos(robotHeadingRadians))
                - (ROBOT_CENTER_TO_TURRET_DY * Math.sin(robotHeadingRadians));

        double turretY = robotPose.getY()
                + (ROBOT_CENTER_TO_TURRET_DX * Math.sin(robotHeadingRadians))
                + (ROBOT_CENTER_TO_TURRET_DY * Math.cos(robotHeadingRadians));

        double diffX = GOAL_X - turretX;
        double diffY = GOAL_Y - turretY;

        double pivotDist = Math.hypot(diffX, diffY);
        distanceToGoal = pivotDist - TURRET_PIVOT_TO_BARREL;

        double targetDiffX = diffX;
        double targetDiffY = diffY;

        double vX = robotVelocity.getXComponent();
        double vY = robotVelocity.getYComponent();

        double aX = robotAcceleration.getXComponent();
        double aY = robotAcceleration.getYComponent();

        double baseTPS = calculateTargetTPS(distanceToGoal);

        if(getTurretState() == TurretState.AUTO){
            double estimatedRingSpeed = baseTPS * TPS_TO_INCHES_PER_SEC;

            if(estimatedRingSpeed < 100) estimatedRingSpeed = 100;

            double flightTime = distanceToGoal / estimatedRingSpeed;

            double virtualGoalX = GOAL_X - (vX * flightTime * INERTIA_GAIN) - (aX * flightTime * ACCEL_FEEDFORWARD_GAIN);
            double virtualGoalY = GOAL_Y - (vY * flightTime * INERTIA_GAIN) - (aY * flightTime * ACCEL_FEEDFORWARD_GAIN);

            targetDiffX = virtualGoalX - turretX;
            targetDiffY = virtualGoalY - turretY;
        }

        double target_angle = 0;
        if(getTurretState() == TurretState.AUTO){
            double targetFieldDegrees = Math.toDegrees(Math.atan2(targetDiffY, targetDiffX));
            target_angle = normalizeAngle(targetFieldDegrees - robotHeadingDegrees + pad_offset);

            target_angle = Math.max(min_turret_angle, Math.min(target_angle, max_turret_angle));
        }
        else if (getTurretState() == TurretState.FRONT) {
            target_angle = Math.max(min_turret_angle, Math.min(test_target, max_turret_angle));
        }

        if(getShooterState() == ShooterState.SHOOT){
            double aimAngleRad = Math.atan2(targetDiffY, targetDiffX);

            double robotVelIntoShot = (vX * Math.cos(aimAngleRad))
                    + (vY * Math.sin(aimAngleRad));

            double robotAccelIntoShot = (aX * Math.cos(aimAngleRad)) + (aY * Math.sin(aimAngleRad));

            double effectiveVelocity = robotVelIntoShot + (robotAccelIntoShot * SHOOTER_ACCEL_GAIN);

            double correctionFactor;

            if (effectiveVelocity > 0) {
                correctionFactor = TPS_TO_INCHES_FORWARD;
            } else {
                correctionFactor = TPS_TO_INCHES_BACKWARD;
            }

            double TPSCorrection = effectiveVelocity / correctionFactor;

            targetTPS = baseTPS - TPSCorrection;

            shooterPIDF.setPIDF(sP, 0, 0, 0);
            feedforward = new SimpleMotorFeedforward(skStatic, skVelocity, skAcceleration);

            double currentTPS = leftShooterMotor.getVelocity();

            double pidCorrection = shooterPIDF.calculate(currentTPS, targetTPS);
            double ffOutput = feedforward.calculate(targetTPS)*(12.0 / voltage);

            double finalPower = pidCorrection + ffOutput;

            if (finalPower > 1.0) finalPower = 1.0;
            if (finalPower < 0.0) finalPower = 0.0;

            leftShooterMotor.setPower(finalPower);
            rightShooterMotor.setPower(finalPower);
        }
        else if(getShooterState() == ShooterState.TEST){
            shooterPIDF.setPIDF(sP, sI, sD, 0);
            feedforward = new SimpleMotorFeedforward(skStatic, skVelocity, skAcceleration);

            targetTPS = test_TPS;

            double currentTPS = rightShooterMotor.getVelocity();

            double pidCorrection = shooterPIDF.calculate(currentTPS, targetTPS);
            double ffOutput = feedforward.calculate(targetTPS)*(12.0 / voltage);

            double finalPower = pidCorrection + ffOutput;

            if (finalPower > 1.0) finalPower = 1.0;
            if (finalPower < 0.0) finalPower = 0.0;

            leftShooterMotor.setPower(finalPower);
            rightShooterMotor.setPower(finalPower);
        }
        else if(getShooterState() == ShooterState.IDLE){
            shooterPIDF.setPIDF(sP, sI, sD, 0);
            feedforward = new SimpleMotorFeedforward(skStatic, skVelocity, skAcceleration);

            targetTPS = 0;

            double currentTPS = rightShooterMotor.getVelocity();

            double pidCorrection = shooterPIDF.calculate(currentTPS, targetTPS);
            double ffOutput = feedforward.calculate(targetTPS)*(12.0 / voltage);

            double finalPower = pidCorrection + ffOutput;

            if (finalPower > 1.0) finalPower = 1.0;
            if (finalPower < 0.0) finalPower = 0.0;

            leftShooterMotor.setPower(finalPower);
            rightShooterMotor.setPower(finalPower);
        }
        else if(getShooterState() == ShooterState.POWER){
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);
        }

        if(getHoodState() == HoodState.AUTO){
            double targetHoodPos = calculateTargetHood(distanceToGoal);
            hoodServo.setPosition(targetHoodPos);
        }
        else if (getHoodState() == HoodState.TEST) {
            double targetHoodPos = calculateTestTargetHood();
            hoodServo.setPosition(targetHoodPos);
        }

        double servoPosition = angleToServoPos(target_angle);
        leftTurretServo.setPosition(servoPosition);
        rightTurretServo.setPosition(servoPosition);


        if(DEBUG_DRAWING) {
            double centeredX = robotPose.getX() - 72.0;
            double centeredY = robotPose.getY() - 72.0;
            double centeredGoalX = GOAL_X - 72.0;
            double centeredGoalY = GOAL_Y - 72.0;

            double drawX = -centeredY;
            double drawY = centeredX;
            double drawHeading = robotPose.getHeading() + (Math.PI / 2.0);

            double drawGoalX = -centeredGoalY;
            double drawGoalY = centeredGoalX;

            double currentTurretRad = Math.toRadians(-target_angle);

            double drawTurretRad = drawHeading + currentTurretRad + Math.PI;

            Canvas field = packet.fieldOverlay();

            field.setStroke("blue");
            field.strokeCircle(drawX, drawY, 9.0);

            double rHeadX = drawX + (9.0 * Math.cos(drawHeading));
            double rHeadY = drawY + (9.0 * Math.sin(drawHeading));
            field.strokeLine(drawX, drawY, rHeadX, rHeadY);

            double tX = drawX
                    + (ROBOT_CENTER_TO_TURRET_DX * Math.cos(drawHeading))
                    - (ROBOT_CENTER_TO_TURRET_DY * Math.sin(drawHeading));

            double tY = drawY
                    + (ROBOT_CENTER_TO_TURRET_DX * Math.sin(drawHeading))
                    + (ROBOT_CENTER_TO_TURRET_DY * Math.cos(drawHeading));

            field.setStroke("red");
            field.strokeCircle(tX, tY, 3.0);

            double barrelLen = TURRET_PIVOT_TO_BARREL;

            double bX = tX + (barrelLen * Math.cos(drawTurretRad));
            double bY = tY + (barrelLen * Math.sin(drawTurretRad));

            field.strokeLine(tX, tY, bX, bY);

            field.fillCircle(bX, bY, 2.0);

            field.setFill("green");
            field.fillCircle(drawGoalX, drawGoalY, 4.0);

            field.setStroke("#00FF0088");
            field.strokeLine(bX, bY, drawGoalX, drawGoalY);
        }

    }

}
