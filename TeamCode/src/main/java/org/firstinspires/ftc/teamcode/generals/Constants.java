package org.firstinspires.ftc.teamcode.generals;

public final class Constants {
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 46.7532467532;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 54.5454545455;

    public static final double LIFT_ENCODER_TICKS_PER_INCH = 1;

    public static final double CLAW_OPEN = 0;
    public static final double CLAW_CLOSE = 0;
    public static final double CLAW_HEIGHT = 0;

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
