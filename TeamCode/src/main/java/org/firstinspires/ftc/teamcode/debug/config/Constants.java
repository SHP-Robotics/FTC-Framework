package org.firstinspires.ftc.teamcode.debug.config;

public final class Constants {
    // Oh god no
    public static final double TILE_LENGTH = 23.3437;

    // Wheels
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 32.8741735;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 42.36 * 48 / 50;

    public static final double MINIMUM_VOLTAGE_APPLIED = 0.0654;

    // Outtake
    public static final double OUTTAKE_STARTING = 0.9;
    public static final double OUTTAKE_NEUTRAL = 0.66;
    public static final double OUTTAKE_LOWERED = 0.58;
    public static final double OUTTAKE_ACTIVE = 0.51;
    public static final double OUTTAKE_HIDDEN = 0.27;

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
