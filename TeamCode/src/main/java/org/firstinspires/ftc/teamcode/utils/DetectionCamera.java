package org.firstinspires.ftc.teamcode.utils;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;

public class DetectionCamera {
    public CameraName camera;
    public VisionPortal visionPortal;
    public DetectionPipeline pipeline;

    public DetectionCamera(CameraName camera){
        this.camera = camera;
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);
        pipeline = new DetectionPipeline();
        builder.addProcessor(pipeline);

        visionPortal = builder.build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}