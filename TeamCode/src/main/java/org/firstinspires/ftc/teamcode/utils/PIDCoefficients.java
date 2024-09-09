package org.firstinspires.ftc.teamcode.utils;

public class PIDCoefficients {
    double kD;
    double kP;
    double kI;
    private double integral = 0; // error sum for kI

    private double lastError = 0;
    private double errorThreshold = 0.03; // If error is less than this, then the error is considered 0
    public PIDCoefficients(double kP, double kD, double kI) {
        this.kD = kD;
        this.kP = kP;
        this.kI = kI;
    }

    public void setErrorThreshold(double errorThreshold) {
        this.errorThreshold = errorThreshold;
    }

    double calculateError(double target, double actual) {
        double error = target - actual;
        if (Math.abs(error) < errorThreshold) {
            error = 0;
        }
        return error;
    }
    public double update(double target, double actual, double dt) {
        double error = calculateError(target, actual);
        double derivative = (error - lastError) / dt;
        lastError = error;

        integral += error * dt;
        return kP * error + kD * derivative + kI * integral;
    }
    public double update(double error, double dt) {
        double derivative = error / dt;
        return kP * error + kD * derivative;
    }
}
