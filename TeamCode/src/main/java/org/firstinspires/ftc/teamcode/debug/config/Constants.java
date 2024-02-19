package org.firstinspires.ftc.teamcode.debug.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class Constants {
    // Pure Pursuit
    // ** means tunable
    // * means approximate
    public static final double MECANUM_WIDTH = 15; // Inches each wheel travels per rotation *
    public static final double ODOMETRY_TICKS_PER_INCH = 336.878636635*4*9.8/(3*12); // # of ticks per dead wheel travelling 1" **
    public static final double ODOMETRY_WIDTH = 13.822861635923072; // Rotational diameter **
    public static final double CIRCULAR_RATIO = 0.7658864198272942; // Cancels out rotation causing movement on the x-axis **

    public static final double followRadius = 2; // Follow radius *
    public static final double positionBuffer = 0.05; // Maximum positional error admitted *
    public static final double rotationBuffer = Math.toRadians(5); // Maximum rotational error admitted *

    public static final double tanhPace = 1; // Acceleration constant *
    public static final double minimumTanh = 0.05; // Minimum path following speed *
    public static final double maximumTanh = 0.3; // Maximum path following speed *

    public static final DcMotorSimple.Direction leftEncoderDirection = DcMotorSimple.Direction.FORWARD; // **
    public static final DcMotorSimple.Direction rightEncoderDirection = DcMotorSimple.Direction.FORWARD; // **
    public static final DcMotorSimple.Direction centerEncoderDirection = DcMotorSimple.Direction.REVERSE; // **

    // Wheels
    public static final DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE; // **
    public static final DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD; // **
    public static final DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE; // **
    public static final DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.REVERSE; // **

    // Basic movement library (not Pure Pursuit)
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 32.8741735;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 42.36 * 48 / 50;

    public static final double MINIMUM_VOLTAGE_APPLIED = 0.0654;

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
