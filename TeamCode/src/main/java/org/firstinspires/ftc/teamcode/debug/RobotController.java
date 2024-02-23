package org.firstinspires.ftc.teamcode.debug;

public abstract class RobotController {
    public abstract void driveParams(double x, double y, double r);
    public abstract void driveParams(double x, double y, double r, double[] bottleneckSpeeds);
    public abstract void driveFieldParams(double x, double y, double r, double gyro);
    public abstract void driveFieldParams(double x, double y, double r, double gyro, double[] bottleneckSpeeds);
    public abstract boolean isBusy();
    public abstract void waitUntilCompletion();
    public abstract void deactivate();
}
