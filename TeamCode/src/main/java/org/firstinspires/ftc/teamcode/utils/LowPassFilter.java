package org.firstinspires.ftc.teamcode.utils;

public class LowPassFilter {
    private double lastEstimate = 0;
    private double alpha = 0.2;

    public LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    public double estimate(double measurement) {
        double currentEstimate = (alpha * measurement) + ((1 - alpha) * lastEstimate);
        lastEstimate = currentEstimate;
        return currentEstimate;
    }

    public void reset(double measurement) {
        lastEstimate = measurement;
    }
}