package org.firstinspires.ftc.teamcode.debug.config;

public final class Constants {
    // Wheels
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 42.74;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 52.36;
    public static final double MINIMUM_VOLTAGE_APPLIED = 0.044;

    // Gear Shifting PID Controller
    public static final double KP = 1.00;
    public static final double KI = 0.00;
    public static final double KD = 0.00;

    // Dual Pixel Storage
    public static final double DEEP_PIXEL_CLAW_HEIGHT = 0;
    public static final double SHALLOW_PIXEL_CLAW_HEIGHT = 0;

    // Four Bar
    public static final double BEHIND_DUAL_PIXEL_STORAGE_POSITION = 0;
    public static final double DUAL_PIXEL_STORAGE_POSITION = 0;
    public static final double OUTTAKE_POSITION = 0;

    // Lift
    public static final double LIFT_ENCODER_TICKS_PER_INCH = 1;

    public static final double LOW_BONUS_HEIGHT = 0;
    public static final double MEDIUM_BONUS_HEIGHT = 0;
    public static final double HIGH_BONUS_HEIGHT = 0;

    // Claw
    public static final double CLAW_OPEN = 0;
    public static final double CLAW_CLOSE = 1;
    public static final double CLAW_HEIGHT = 0;

    // TODO: Find these values
    // Camera Focal Length, used for to estimate Object Location
    // FOCAL_LENGTH = (PERCEIVED_WIDTH x DISTANCE) / WIDTH
    // public static final double BACKDROP_WIDTH = 25.625;
    // public static final double FOCAL_LENGTH = 1;
    public static final double APRIL_TAG_POSITION_CORRECTION = 1.2676056338;

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
