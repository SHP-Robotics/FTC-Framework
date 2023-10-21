package org.firstinspires.ftc.teamcode.shplib.hardware.drive.localizer;

public class Point {
    private final double[] setpoints;

    public Point(double straight) {
        this.setpoints = new double[]{
                straight,
                straight,
                straight,
                straight
        };
    }

    public Point(double left, double right) {
        this.setpoints = new double[]{
                left,
                left,
                right,
                right
        };
    }

    public Point(double leftFront, double leftRear, double rightFront, double rightRear) {
        this.setpoints = new double[]{
                leftFront,
                leftRear,
                rightFront,
                rightRear
        };
    }

    public Point with(Point other) {
        return new Point(
                setpoints[0] + other.setpoints[0],
                setpoints[1] + other.setpoints[1],
                setpoints[2] + other.setpoints[2],
                setpoints[3] + other.setpoints[3]
        );
    }

    public Point reversed() {
        return new Point(
                -setpoints[0],
                -setpoints[1],
                -setpoints[2],
                -setpoints[3]
        );
    }

    public double[] getSetpoints() {
        return setpoints;
    }
}
