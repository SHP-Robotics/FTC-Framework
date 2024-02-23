package org.firstinspires.ftc.teamcode.debug.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class Constants {
    // Pure Pursuit
    // ** means tunable
    // * means approximate
    public static final double MECANUM_WIDTH = 15; // Inches each wheel travels per rotation **
    public static final double ODOMETRY_TICKS_PER_INCH = 336.877962878; // ticks per rev / circumference **
    public static double ODOMETRY_WIDTH = 23.36825; // Rotational diameter **
    public static double FORWARD_OFFSET = -0.0294066; // Cancels out rotation causing movement on the x-axis after rotations **
    public static double MAX_VELOCITY = 131; // **

    public static final double followRadius = 2; // Follow radius *
    public static final double headingCorrected = Math.toRadians(1); // in radians, maximum acceptable directional error *
    public static final double positionBuffer = 0.5; // Default positional error admitted *
    public static final double rotationBuffer = Math.toRadians(5); // Default rotational error admitted *

    public static final double tanhPace = 0.8; // Acceleration constant *
    public static final double minimumTanh = 0.04; // Minimum path following speed *
    public static final double maximumTanh = 0.4; // Maximum path following speed *

    public static final DcMotorSimple.Direction leftEncoderDirection = DcMotorSimple.Direction.REVERSE; // **
    public static final DcMotorSimple.Direction rightEncoderDirection = DcMotorSimple.Direction.FORWARD; // **
    public static final DcMotorSimple.Direction centerEncoderDirection = DcMotorSimple.Direction.FORWARD; // **
    // Wheels
    public static final DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE; // **
    public static final DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD; // **
    public static final DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE; // **
    public static final DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.FORWARD; // **

    public static double[] powers = new double[]{
            1,
            0.97,
            1,
            0.97
    };

    // Basic movement library (not Pure Pursuit)
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 32.8741735;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 42.36 * 48 / 50;

    public static final double MINIMUM_VOLTAGE_APPLIED = 0.0654;

    // Outtake
    public static final double OUTTAKE_STARTING = 0.81;
    public static final double OUTTAKE_NEUTRAL = 0.69;
    public static final double OUTTAKE_LOWERED = 0.43;
    public static final double OUTTAKE_ACTIVE = 0.437;
    public static final double OUTTAKE_HIDDEN = 0.213;

    // Claw
    public static final double CLAW_OPEN = 0.5936;
    public static final double CLAW_CLOSE = 0.57;

    // Airplane
    public static final double AIRPLANE_RELEASE = 0.07;
    public static final double AIRPLANE_HOLD = 0.41152;

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
