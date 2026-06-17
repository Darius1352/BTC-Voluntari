package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDController {
    private PIDCoefficients PID_COEFFICIENTS;

    public double tolerance;
    public double target;
    public double errorSum = 0;
    public double lastUpdateError;
    public double lastUpdateTime;
    public double upperBound = 1e9;
    public double lowerBound = -1e9;

    public PIDController(PIDCoefficients pidCoefficients){
        this.PID_COEFFICIENTS = pidCoefficients;
    }

    public PIDController(double kp, double ki, double kd){
        this.PID_COEFFICIENTS = new PIDCoefficients(kp,ki,kd);
    }

    public void setPID_COEFFICIENTS(PIDCoefficients PID_COEFFICIENTS) {
        this.PID_COEFFICIENTS = PID_COEFFICIENTS;
    }

    public void setPID_COEFFICIENTS(double kp, double ki, double kd){
        this.PID_COEFFICIENTS = new PIDCoefficients(kp,ki,kd);
    }

    public PIDCoefficients getPID_COEFFICIENTS(){
        return PID_COEFFICIENTS;
    }

    public PIDController setTarget(double target){
        this.target = Math.min(upperBound, Math.max(lowerBound,target));
        return this;
    }

    public double getTarget(){
        return target;
    }

    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    public boolean atTarget(){
        return Math.abs(lastUpdateError) <= tolerance;
    }

    public void setUpperBound(double upperBound){
        this.upperBound = upperBound;
    }

    public void setLowerBound(double lowerBound){
        this.lowerBound = lowerBound;
    }

    public double getLastError(){
        return lastUpdateError;
    }

    public double updateError(double error){
        double calculatedValue = 0;
        double errorDerivative = 0;
        double deltaError = error - lastUpdateError;
        double deltaTime = lastUpdateTime - System.nanoTime() / 1e9;

        lastUpdateError = error;
        lastUpdateTime = System.nanoTime() / 1e9;

        if (lastUpdateTime != -1){
            errorDerivative = deltaError / deltaTime;
            errorSum += (deltaError + lastUpdateError) / 2 * deltaTime;
        }
        double proportionalPower = error * PID_COEFFICIENTS.p;
        double derivativePower = errorDerivative * PID_COEFFICIENTS.d;
        double integralPower = errorSum * PID_COEFFICIENTS.i;

        calculatedValue = proportionalPower + derivativePower + integralPower;
        return calculatedValue;
    }

    public double update(double currentValue){
        double calculatedValue = 0;
        double errorDerivative = 0;
        double error = target - currentValue;
        double deltaError = error - lastUpdateError;
        double deltaTime = lastUpdateTime - System.nanoTime() / 1e9;

        lastUpdateError = error;
        lastUpdateTime = System.nanoTime() / 1e9;

        if (lastUpdateTime != -1){
            errorDerivative = deltaError / deltaTime;
            errorSum += (deltaError + lastUpdateError) / 2 * deltaTime;
        }
        double proportionalPower = error * PID_COEFFICIENTS.p;
        double derivativePower = errorDerivative * PID_COEFFICIENTS.d;
        double integralPower = errorSum * PID_COEFFICIENTS.i;

        calculatedValue = proportionalPower + derivativePower + integralPower;
        return calculatedValue;
    }



}