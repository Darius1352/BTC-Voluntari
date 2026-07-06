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
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class DetectionPipeline implements VisionProcessor, CameraStreamSource {
    // Focal length and known width for distance calculation.
    // KNOWN_WIDTH: the actual physical width of your known object (in cm).
    private static final double KNOWN_WIDTH = 3.7;   // in cm
    private static final double FOCAL_LENGTH = 230;    // calibrated focal length (in pixel units)

    // Configurable HSV Color Ranges for Red, Yellow, and Blue.
    private static final Map<String, Scalar[]> HSV_RANGES = new HashMap<>();
    static {
        HSV_RANGES.put("red", new Scalar[]{ new Scalar(112, 130, 120), new Scalar(175, 255, 255) });
        HSV_RANGES.put("yellow", new Scalar[]{ new Scalar(90, 100, 120), new Scalar(108, 255, 255) });
        HSV_RANGES.put("blue", new Scalar[]{ new Scalar(0, 70, 40), new Scalar(45, 255, 255) });
    }

    // Additional configurable filtering parameters.
    public static double MIN_CONTOUR_AREA = 1500; // in pixels
    // Minimum candidate width and height in cm (physical size you expect at a typical distance)
    public static double minWidth_cm = 3.1;
    public static double minHeight_cm = 7.8;
    // Typical distance at which you expect to see the object (in cm)
    public static double typicalDistance_cm = 30.0;
    // Expected aspect ratio range (for your object)
    public static double[] ASPECT_RATIO_RANGE = {2.1, 2.5};
    public static int morphKernelSize = 7;  // kernel size for morphological operations

    // Flag: if true, assume the input frame is in BGR; if false, assume RGB.
    public static boolean useBGR = true;

    // Holds the latest processed Bitmap for streaming.
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    // Stores the last computed normalized center offsets (x,y) from the image center.
    private DistanceXY lastDistanceXY = new DistanceXY(0, 0);

    // Stores the color of the last candidate detected.
    private String lastColor = "";

// Helper class to store candidate information.
public static class Candidate {
    MatOfPoint contour;
    RotatedRect rect;
    public double area;
    double aspectRatio;
    String color;
    Candidate(MatOfPoint contour, RotatedRect rect, double area, double aspectRatio, String color) {
        this.contour = contour;
        this.rect = rect;
        this.area = area;
        this.aspectRatio = aspectRatio;
        this.color = color;
    }
}

    // New inner class for storing normalized (x,y) offsets.
    public static class DistanceXY {
        private double x;
        private double y;
        public DistanceXY(double x, double y) {
            this.x = x;
            this.y = y;
        }
        public double getX() { return x; }
        public double getY() { return y; }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // No additional initialization required.
    }

    private void incrementTestVar() {
        // Optional: debug printing if needed.
    }

    private void drawDecorations(Mat image) {
        Imgproc.putText(image,
                "Limelight Vision Processing",
                new Point(10, 230),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5,
                new Scalar(0, 255, 0),
                1);
    }

/**
 * Filters contours by area, aspect ratio, and minimum physical dimensions.
 * Converts the desired minimum physical dimensions (cm) into pixels using:
 * pixelThreshold = (minDimension_cm * FOCAL_LENGTH) / typicalDistance_cm.
 */
private List<Candidate> filterContoursByAspect(List<MatOfPoint> contours, double minArea, double[] aspectRatioRange, String color) {
    List<Candidate> candidates = new ArrayList<>();
    double minWidth_px = (minWidth_cm * FOCAL_LENGTH) / typicalDistance_cm;
    double minHeight_px = (minHeight_cm * FOCAL_LENGTH) / typicalDistance_cm;
    for (MatOfPoint cnt : contours) {
        double area = Imgproc.contourArea(cnt);
        if (area < MIN_CONTOUR_AREA || area < minArea)
            continue;
        MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
        RotatedRect rect = Imgproc.minAreaRect(cnt2f);
        double w = rect.size.width;
        double h = rect.size.height;
        double angle = rect.angle;
        if (w < h) {
            double temp = w;
            w = h;
            h = temp;
            angle += 90;
        }
        if (w < minWidth_px || h < minHeight_px)
            continue;
        double aspectRatio = (h > 0) ? (w / h) : 0;
        if (aspectRatio >= aspectRatioRange[0] && aspectRatio <= aspectRatioRange[1]) {
            candidates.add(new Candidate(cnt, rect, area, aspectRatio, color));
        }
    }
    return candidates;
}

    private Candidate selectBestCandidate(List<Candidate> candidates) {
        if (candidates.isEmpty())
            return null;
        Candidate best = candidates.get(0);
        for (Candidate candidate : candidates) {
            if (candidate.area > best.area) {
                best = candidate;
            }
        }
        return best;
    }
    /**
     * Processes the input frame (in-place) to produce a binary mask and additional detection data.
     * It uses HSV thresholding and morphological operations to generate a pure binary mask.
     * If a candidate object is detected, it draws the candidate's outline and annotations.
     *
     * The outputData array now contains:
     * [detectedFlag, normCenterX, normCenterY, width (cm), height (cm), normAngle, distance (cm), 0]
     *
     * Additionally, the normalized center (x,y) is stored in lastDistanceXY.
     *
     * @param input The original image frame (Mat) to be processed in-place.
     * @param captureTimeNanos Capture timestamp (unused).
     * @return A Map with keys "contour", "annotatedImage", and "outputData".
     */
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Step 1: Preprocess: Blur and convert a clone of the input to HSV.
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(input, blurred, new Size(5, 5), 0);
        Mat imgHSV = new Mat();
        Imgproc.cvtColor(blurred, imgHSV, Imgproc.COLOR_BGR2HSV);

        Candidate bestCandidate = null;
        String bestColor = "";

        // Process each HSV range.
        for (String color : HSV_RANGES.keySet()) {
            Scalar[] thresholds = HSV_RANGES.get(color);
            Scalar lowerHSV = thresholds[0];
            Scalar upperHSV = thresholds[1];

            Mat mask = new Mat();
            Core.inRange(imgHSV, lowerHSV, upperHSV, mask);

            // Apply morphological operations.
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(morphKernelSize, morphKernelSize));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
// Find contours.
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<Candidate> candidates = filterContoursByAspect(contours, MIN_CONTOUR_AREA, ASPECT_RATIO_RANGE, color);
            Candidate candidate = selectBestCandidate(candidates);
            if (candidate != null) {
                if (bestCandidate == null || candidate.area > bestCandidate.area) {
                    bestCandidate = candidate;
                    bestColor = color;
                }
            }
            mask.release();
            hierarchy.release();
        }

        // Prepare output data array.
        double[] outputData = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

        int imgWidth = input.width();
        int imgHeight = input.height();

        if (bestCandidate != null) {
            RotatedRect rect = bestCandidate.rect;
            double x = rect.center.x;
            double y = rect.center.y;
            double w = rect.size.width;
            double h = rect.size.height;
            double angle = rect.angle;
            if (w < h) {
                double temp = w;
                w = h;
                h = temp;
                angle += 90;
            }
            // Draw candidate outline on the input.
            Point[] boxPoints = new Point[4];
            rect.points(boxPoints);
            MatOfPoint box = new MatOfPoint();
            box.fromArray(boxPoints);
            List<MatOfPoint> boxContour = new ArrayList<>();
            boxContour.add(box);
            Imgproc.drawContours(input, boxContour, 0, new Scalar(255, 255, 255), 2);

            // Distance calculation (using the long side in pixels).
            double distance = (w != 0) ? (KNOWN_WIDTH * FOCAL_LENGTH) / w : 0;

            // Convert candidate dimensions from pixels to centimeters.
            double w_cm = (w * distance) / FOCAL_LENGTH;
            double h_cm = (h * distance) / FOCAL_LENGTH;

            // Compute normalized center coordinates.
            double normCenterX = x / imgWidth;
            double normCenterY = y / imgHeight;
            // Normalize angle to [0,1] (assuming angle in [0,180]).
            double normAngle = angle / 180.0;

            // Draw the centroid.
            Moments M = Imgproc.moments(bestCandidate.contour);
            int cx, cy;
            if (M.get_m00() != 0) {
                cx = (int)(M.get_m10() / M.get_m00());
                cy = (int)(M.get_m01() / M.get_m00());
            } else {
                cx = (int)x;
                cy = (int)y;
            }
            Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(255, 255, 255), -1);
// Annotate with detection info.
            Imgproc.putText(input,
                    "D: " + String.format("%.2f", distance) + " cm",
                    new Point(x - 20, y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(255, 255, 255),
                    1);
            Imgproc.putText(input,
                    "Angle: " + String.format("%.2f", angle),
                    new Point(x - 20, y + 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(255, 255, 255),
                    1);
            Imgproc.putText(input,
                    "Color: " + bestColor,
                    new Point(x - 20, y + 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(255, 255, 255),
                    1);

            outputData = new double[]{1, normCenterX, normCenterY, Math.round(w_cm * 100.0) / 100.0, Math.round(h_cm * 100.0) / 100.0, Math.round(normAngle * 100.0) / 100.0, Math.round(distance * 100.0) / 100.0, 0};

            // Store the normalized center in lastDistanceXY.
            lastDistanceXY = new DistanceXY(normCenterX, normCenterY);
            // Store the detected candidate's color.
            lastColor = bestColor;
        } else {
            input.setTo(new Scalar(0,0,0));
            lastDistanceXY = new DistanceXY(0,0);
            lastColor = "";
        }

        incrementTestVar();
        drawDecorations(input);
        // Convert the annotated input to a Bitmap for streaming.
        Bitmap bmp = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bmp);
        lastFrame.set(bmp);

        // Package the results into a Map.
        Map<String, Object> result = new HashMap<>();
        result.put("contour", bestCandidate != null ? bestCandidate.contour : new Mat());
        result.put("annotatedImage", input);
        result.put("outputData", outputData);

        return result;
    }
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(consumer -> consumer.accept(lastFrame.get()));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // No additional onDrawFrame actions.
    }

    // Returns the normalized (x,y) offset from the image center.
    public DistanceXY getSampleDistance() {
        return lastDistanceXY;
    }

    // Returns the color of the detected candidate (empty string if none detected).
    public String getColor() {
        return lastColor;
    }

    // Returns true if a candidate was detected in the last frame.
    public boolean isDetected() {
        return !lastColor.isEmpty();
    }
}