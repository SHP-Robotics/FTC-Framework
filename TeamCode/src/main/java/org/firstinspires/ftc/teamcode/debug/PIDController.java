package org.firstinspires.ftc.teamcode.debug;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;

    private double min;
    private double max;

    private double errorSum;
    private double lastError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.errorSum = 0;
        this.lastError = 0;
    }

    public double getOutput(double error) {
        errorSum += error;
        double errorChange = error - lastError;
        lastError = error;

        return kP * error + kI * errorSum + kD * errorChange;
    }

    public void reset() {
        errorSum = 0;
        lastError = 0;
    }
}
