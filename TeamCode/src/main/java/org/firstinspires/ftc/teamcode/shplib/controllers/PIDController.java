package org.firstinspires.ftc.teamcode.shplib.controllers;

import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

public class PIDController {
    /*private*/ double kP, kI, kD;

    private double period = 0.0; // seconds
    private double integralSum = 0.0;
    private double previousError = 0.0;

    private double previousTime = 0.0;

    private double error = 0.0;
    private double errorTolerance = 1.0;

    public PIDController(double kP) {
        setGains(kP, 0.0, 0.0);
    }

    public PIDController(double kP, double kI, double kD) {
        setGains(kP, kI, kD);
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    private void setPeriod(double period) {
        this.period = period;
    }

    public double getPeriod() {
        return period;
    }

    public void setErrorTolerance(double errorTolerance) {
        this.errorTolerance = errorTolerance;
    }

    public double calculate(double measurement, double setpoint) {
        double output = 0;

        error = setpoint - measurement;

        if (kI > 0 || kD > 0) {
            setPeriod(Clock.elapsed(previousTime));
            previousTime = Clock.now();
        }

        // Proportional
        if (kP > 0) {
            output += (kP * error);
        }

        // + Integral
        if (kI > 0) {
            integralSum = integralSum + (error * period);
            output += (kI * integralSum);
        }

        // + Derivative
        if (kD > 0) {
            double derivative = (error - previousError) / period;
            output += (kD * derivative);
            previousError = error;
        }

        return output;
    }

    public boolean atSetpoint() {
        return Math.abs(error) < errorTolerance;
    }

    public void resetIntegralSum() {
        integralSum = 0.0;
    }
}
