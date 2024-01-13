package org.firstinspires.ftc.teamcode.debug.config;

public final class Constants {
    // Oh god no
    public static final double TILE_LENGTH = 23.3437;

    // Wheels
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 32.8741735;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 42.36 * 48 / 50;

    public static final double MINIMUM_VOLTAGE_APPLIED = 0.0654;

    // Outtake
//    public static final double OUTTAKE_STARTING = 0.9;
//    public static final double OUTTAKE_NEUTRAL = 0.66;
//    public static final double OUTTAKE_LOWERED = 0.56;
//    public static final double OUTTAKE_ACTIVE = 0.50;
//    public static final double OUTTAKE_HIDDEN = 0.27;

    // Claw
    public static final double CLAW_OPEN = 0;
    public static final double CLAW_CLOSE = 1;

    // Camera
    // TODO: tune servo position for camera
    public enum CameraMode {
        DETECTING(0),
        SONAR(0),
        CLAW(0);

        private double position;

        private CameraMode(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

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
