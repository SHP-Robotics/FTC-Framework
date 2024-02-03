package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public class Position2D {
    private double x;
    private double y;
    private double headingRadians;

    public Position2D(double x, double y) {
        this.x = x;
        this.y = y;
        this.headingRadians = 0;
    }

    public Position2D(double x, double y, double headingRadians) {
        this.x = x;
        this.y = y;
        this.headingRadians = clampRadians(headingRadians);
    }

    private static double clampRadians(double headingRadians) {
        while (headingRadians < -Math.PI) {
            headingRadians += 2*Math.PI;
        }

        while (headingRadians > Math.PI) {
            headingRadians -= 2 * Math.PI;
        }

        return headingRadians;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingRadians() {
        return headingRadians;
    }

    public void add(Position2D position2D, boolean clamp) {
        this.x += position2D.getX();
        this.y += position2D.getY();
        this.headingRadians = this.headingRadians + position2D.getHeadingRadians();
        if (clamp) {
            this.headingRadians = clampRadians(this.headingRadians);
        }
    }

    public double dist(Position2D position2D) {
        double x2 = position2D.getX();
        double y2 = position2D.getY();
        return Math.sqrt(((this.x - x2) * (this.x - x2)) + ((this.y - y2) * (this.y - y2)));
    }
}
