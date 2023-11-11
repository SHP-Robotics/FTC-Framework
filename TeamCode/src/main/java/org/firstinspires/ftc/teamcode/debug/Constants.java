package org.firstinspires.ftc.teamcode.debug;

public final class Constants {
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_FORWARD = 46.7532467532;
    public static final double WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS = 54.5454545455;
<<<<<<< Updated upstream:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/debug/Constants.java
=======
    public static final double MINIMUM_VOLTAGE_APPLIED = 0.07;

    // Gear Shifting PID Controller
    public static final double KP = 0.1;
    public static final double KI = 0.00;
    public static final double KD = 0.01;
>>>>>>> Stashed changes:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/debug/config/Constants.java

    public static final double LIFT_ENCODER_TICKS_PER_INCH = 1;

    public static final double CLAW_OPEN = 0;
    public static final double CLAW_CLOSE = 0;
    public static final double CLAW_HEIGHT = 0;

    //FTC-AI
    public static final float UNIT_LENGTH = 20;
    public static final float UNIT_ROTATION = 180;

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
