package org.firstinspires.ftc.teamcode.debug;

public interface RobotController {
    void driveParams(double x, double y, double r);
    void driveParams(double x, double y, double r, double[] bottleneckSpeeds);
    void driveFieldParams(double x, double y, double r, double gyro);
    void driveFieldParams(double x, double y, double r, double gyro, double[] bottleneckSpeeds);
    boolean isBusy();
    void waitUntilCompletion();
    void deactivate();
}
