package org.firstinspires.ftc.teamcode.generals;

public final class Constants {
    public static final double WHEEL_ENCODER_TICKS_PER_INCH = 1;

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
