package org.firstinspires.ftc.teamcode.lib.controller;

import edu.wpi.first.math.MathUtil;

public class SquIDController {
    private double squP;

    private boolean continuous;
    private double maximumInput;
    private double minimumInput;

    public SquIDController(double squP) {
        this.squP = squP;
    }

    public void setP(double p) {
        squP = p;
    }

    public double calculate(double squP, double setpoint, double measurement) {
        double error;
        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            error = MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            error = setpoint - measurement;
        }

        return squP * Math.sqrt(Math.abs(error)) * Math.signum(error);
    }

    public double calculate(double setpoint, double measurement) {
        return calculate(squP, setpoint, measurement);
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;

        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    public static double calculateStatic(double squP, double setpoint, double measurement) {
        double error = setpoint - measurement;
        return squP * Math.sqrt(Math.abs(error)) * Math.signum(error);
    }

    public static double calculateStatic(double squP, double setpoint, double measurement, double maximumInput, double minimumInput) {
        double error;

        double errorBound = (maximumInput - minimumInput) / 2.0;
        error = MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound);

        return squP * Math.sqrt(Math.abs(error)) * Math.signum(error);
    }
}
