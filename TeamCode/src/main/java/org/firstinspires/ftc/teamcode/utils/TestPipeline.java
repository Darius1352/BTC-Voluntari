package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class TestPipeline extends OpenCvPipeline implements CameraStreamSource {

    // Holds the most recent frame as a Bitmap.
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    // This Mat clone will be used for display.
    private Mat matClone = new Mat();

    /**
     * Processes each frame from the camera.
     * In this test pipeline, the frame is simply cloned and returned unmodified.
     *
     * @param frame             The original frame from the camera.
     * @param captureTimeNanos  The capture time in nanoseconds.
     * @return                  The cloned frame (to be streamed).
     */


    /**
     * Supplies the latest frame Bitmap to a consumer (e.g., FTC Dashboard).
     *
     * @param continuation A continuation that supplies a Consumer<Bitmap>.
     */
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public Mat processFrame(Mat input) {
        // Log the mean pixel value of the incoming frame
        org.opencv.core.Scalar meanScalar = Core.mean(input);
        Log.d("TestPipeline", "Frame mean: " + meanScalar.toString());

        // Clone the frame for display
        matClone = input.clone();

        // Convert the clone into a Bitmap for streaming
        Bitmap b = Bitmap.createBitmap(matClone.width(), matClone.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(matClone, b);
        lastFrame.set(b);

        // Return the clone so that the streamed view shows the original frame
        return matClone;
    }
    public Bitmap getFrame() {
        return lastFrame.get();
    }
}