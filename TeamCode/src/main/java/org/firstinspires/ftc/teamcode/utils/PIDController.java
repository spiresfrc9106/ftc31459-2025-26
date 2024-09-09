package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController{
    ElapsedTime timer = new ElapsedTime();
    PIDCoefficients coeffecients;
    private double integral = 0; // error sum for kI

    private double lastError = 0;
   public PIDController(PIDCoefficients coefficients) {
        this.coeffecients = coefficients;
    }

    public double update(double target, double actual) {
        double dt = timer.seconds();
        double error = coeffecients.calculateError(target, actual);
        double derivative = (error - lastError) / dt;
        lastError = error;

        integral += error * dt;


        timer.reset();
        return coeffecients.kP * error + coeffecients.kD * derivative + coeffecients.kI * integral;
    }
}