package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@Config
public class VisionFusion {

    private final PoseTracker poseUpdater;
    private final Limelight3A limelight;

    private static final double MAX_TRANSLATIONAL_SPEED = 2.0;
    private static final double MAX_ANGULAR_SPEED = Math.toRadians(5.0);
    private static final double MAX_ALLOWED_DISTANCE_ERROR = 8.0;
    private static final double MIN_UPDATE_INTERVAL = 0.5;
    private ElapsedTime updateTimer;

    private static final double FIELD_SIZE_LIMIT = 150.0;
    private static final double UNIT_CONVERSION = 39.3701;
    private static final double FIELD_CENTER_OFFSET = 72.0;

    public static double LL_OFFSET = -180;

    private boolean isLocalized = false;

    // NOU: Salvăm poziția văzută de cameră pentru a o desena pe Dashboard
    private Pose latestVisionPose = null;

    public VisionFusion(HardwareMap hardwareMap, PoseTracker poseUpdater, String limelightName) {
        this.poseUpdater = poseUpdater;
        this.limelight = hardwareMap.get(Limelight3A.class, limelightName);

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        updateTimer = new ElapsedTime();
    }

    public void update() {
        double imuHeadingRad = poseUpdater.getNormalizedIMUHeading();//aici trebuie scos
        limelight.updateRobotOrientation(Math.toDegrees(imuHeadingRad)+LL_OFFSET);//

        double pedroHeadingDeg = Math.toDegrees(poseUpdater.getPose().getHeading());

        // 2. AICI E SECRETUL: Aplicăm un MINUS pentru a inversa sensul de rotație!
        // Faptul că stai cu fața spre Albastru înseamnă că trebuie să adăugăm 180.
        double limelightHeading = -pedroHeadingDeg + 180.0;

        // 3. Normalizăm unghiul între 0 și 360 ca să nu derutăm Limelight-ul
        while (limelightHeading >= 360.0) limelightHeading -= 360.0;
        while (limelightHeading < 0.0) limelightHeading += 360.0;

        // 4. Trimitem unghiul corectat și aliniat!
        limelight.updateRobotOrientation(limelightHeading);

        // ... restul codului rămâne exact la fel ...
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose3D = result.getBotpose_MT2();

            if (botPose3D != null) {
                Pose visionPose = convertToPedroPose(botPose3D, imuHeadingRad);

                if (isValidFieldPosition(visionPose)) {
                    // Salvăm poziția DOAR pentru desen pe telemetrie
                    latestVisionPose = visionPose;

                    // --- LOGICA DE RELOCALIZARE AUTOMATĂ ---
                    if (updateTimer.seconds() >= MIN_UPDATE_INTERVAL) {
                        Vector velocity = poseUpdater.getVelocity();
                        double angularVelocity = poseUpdater.getAngularVelocity();

                        if (velocity != null && velocity.getMagnitude() <= MAX_TRANSLATIONAL_SPEED
                                && Math.abs(angularVelocity) <= MAX_ANGULAR_SPEED) {

                            Pose currentOdometryPose = poseUpdater.getPose();
                            double distError = Math.hypot(currentOdometryPose.getX() - visionPose.getX(),
                                    currentOdometryPose.getY() - visionPose.getY());

                            if (distError > 0.5 && distError < MAX_ALLOWED_DISTANCE_ERROR) {
                                poseUpdater.setCurrentPoseWithOffset(visionPose);
                                isLocalized = true;
                                updateTimer.reset();
                            }
                        }
                    }
                } else {
                    latestVisionPose = null; // Dacă e în afara terenului, ascundem
                }
            }
        } else {
            latestVisionPose = null; // Dacă nu vede niciun tag, ascundem
        }
    }

    public Pose getLatestVisionPose() {
        return latestVisionPose;
    }

    private Pose convertToPedroPose(Pose3D botPose3D, double currentHeading) {
        double xInches = (botPose3D.getPosition().x * UNIT_CONVERSION) + FIELD_CENTER_OFFSET;
        double yInches = (botPose3D.getPosition().y * UNIT_CONVERSION) + FIELD_CENTER_OFFSET;
        return new Pose(xInches, yInches, currentHeading);
    }

    private boolean isValidFieldPosition(Pose p) {
        return p.getX() > -10 && p.getX() < FIELD_SIZE_LIMIT &&
                p.getY() > -10 && p.getY() < FIELD_SIZE_LIMIT;
    }
}