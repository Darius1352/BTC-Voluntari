package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Limelight {

    public Limelight3A limelight;

    public static double camera_angle_deg = 30;
    public static double camera_height_mm = 285;
    public static double target_height = 749.3;


    private LLResult lastResult = null;
    private double tx = 0;
    private double ty = 0;
    private boolean isTargetVisible = false;

    public double calculatedDistance = 0;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void update() {
        lastResult = limelight.getLatestResult();

        if (lastResult != null && lastResult.isValid()) {
            isTargetVisible = true;
            tx = lastResult.getTx();
            ty = lastResult.getTy();

            //d = (h_target - h_cam) / tan(camera_angle + ty)

            calculatedDistance = (target_height - camera_height_mm) / Math.tan(Math.toRadians(camera_angle_deg + ty));

        } else {
            isTargetVisible = false;
        }
    }

    public double getTx() {
        return tx;
    }

    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    public double getCalculatedDistance() {
        return calculatedDistance;
    }

    public void stop() {
        limelight.stop();
    }
}