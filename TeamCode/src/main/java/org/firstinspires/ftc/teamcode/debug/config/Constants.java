package org.firstinspires.ftc.teamcode.debug.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class Constants {
    // Pure Pursuit
    public static final double MECANUM_WIDTH = 12;
    public static final double ODOMETRY_TICKS_PER_INCH = 336.878636635*4*9.8/(3*12);
    public static final double ODOMETRY_WIDTH = 13.822861635923072;
    public static final double CIRCULAR_RATIO = 0.7658864198272942;

    public static final DcMotorSimple.Direction leftEncoderDirection = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction rightEncoderDirection = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction centerEncoderDirection = DcMotorSimple.Direction.REVERSE;

    // Wheels
    public static final DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.REVERSE;

    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 32.8741735;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 42.36 * 48 / 50;

    public static final double MINIMUM_VOLTAGE_APPLIED = 0.0654;

    // Outtake
    public static final double OUTTAKE_STARTING = 0.9;
    public static final double OUTTAKE_NEUTRAL = 0.66;
    public static final double OUTTAKE_LOWERED = 0.58;
    public static final double OUTTAKE_ACTIVE = 0.51;
    public static final double OUTTAKE_HIDDEN = 0.28;

    // Claw
    public static final double CLAW_OPEN = 0.345;
    public static final double CLAW_CLOSE = 0.4;

    // Airplane
    public static final double AIRPLANE_RELEASE = 0.07;
    public static final double AIRPLANE_HOLD = 0.41152;

    // Camera
    // TODO: tune servo position for camera
    public enum CameraMode {
        FACING_TEAM_PROP(0.11),
        SONAR(0),
        FACING_CLAW(0.59);

        private double position;

        CameraMode(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

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
