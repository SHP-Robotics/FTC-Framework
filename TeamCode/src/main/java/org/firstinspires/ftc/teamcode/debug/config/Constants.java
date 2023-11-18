package org.firstinspires.ftc.teamcode.debug.config;

public final class Constants {
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 46.7532467532;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 54.5454545455;
    public static final double MINIMUM_VOLTAGE_APPLIED = 0.044;

    // Gear Shifting PID Controller
    public static final double KP = 1.00;
    public static final double KI = 0.00;
    public static final double KD = 0.00;

    public static final double LIFT_ENCODER_TICKS_PER_INCH = 122;
    public static final double LIFT_HEIGHT = 36;

    public static final double CLAW_OPEN = 0.4;
    public static final double CLAW_CLOSE = 1;
    public static final double CLAW_HEIGHT = 0;

    // TODO: Find these values
    // Camera Focal Length, used for to estimate Object Location
    // FOCAL_LENGTH = (PERCEIVED_WIDTH x DISTANCE) / WIDTH
    public static final double BACKDROP_WIDTH = 25.625;
    public static final double FOCAL_LENGTH = 1;

    public static double setToDomain(double x, double begin, double end) {
        double length = end - begin;
        while (x < begin) {
            x += length;
        }
        while (x > end) {
            x -= length;
        }
        return x;
    }
}
