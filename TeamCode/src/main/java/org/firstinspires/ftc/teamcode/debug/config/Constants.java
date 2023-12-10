package org.firstinspires.ftc.teamcode.debug.config;

public final class Constants {
    // Oh god no
    public static final double TILE_LENGTH = 23.3437;

    // Wheels
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 32.8741735;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 42.36 * 48 / 50;

    public static final double MINIMUM_VOLTAGE_APPLIED = 0.0654;

    // Gear Shifting PID Controller
    public static final double KP = 1.00;
    public static final double KI = 0.00;
    public static final double KD = 0.00;

    // Outtake
    public static final double OUTTAKE_STARTING = 0.9;
    public static final double OUTTAKE_NEUTRAL = 0.77;
    public static final double OUTTAKE_ACTIVE = 0.52;

    // Claw
    public static final double CLAW_OPEN = 0.345;
    public static final double CLAW_CLOSE = 0.4;
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
