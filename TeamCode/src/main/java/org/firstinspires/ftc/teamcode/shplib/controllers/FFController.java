package org.firstinspires.ftc.teamcode.shplib.controllers;

public class FFController {
    private double kV, kA, kS = 0.0;

    public FFController(double kS) {
        this.kS = kS;
    }

    public FFController(double kV, double kS) {
        this.kV = kV;
        this.kS = kS;
    }

    public FFController(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public double calculate(double velocity) {
        return this.calculate(velocity, 0);
    }

    public double calculate(double velocity, double acceleration) {
        return kV * velocity + kA * acceleration;
    }

    public double getStaticOutput(double power) {
        return kS * Math.signum(power);
    }
}
