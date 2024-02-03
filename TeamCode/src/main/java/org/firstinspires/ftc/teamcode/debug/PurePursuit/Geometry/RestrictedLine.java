package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public class RestrictedLine {
    private final double m;
    private final double b;

    // TODO: implement use for restriction in intersections and plugging in
    private double leftRestriction;
    private double rightRestriction;

    public RestrictedLine(double m, double b) {
        this.m = m;
        this.b = b;
    }

    public RestrictedLine(double x1, double y1, double x2, double y2) {
        this.m = (y2-y1)/(x2-x1);
        this.b = y1-(m*x1);
    }

    public void setLeftRestriction(double leftRestriction) {
        this.leftRestriction = leftRestriction;
    }

    public void setRightRestriction(double rightRestriction) {
        this.rightRestriction = rightRestriction;
    }

    public double getM() {
        return this.m;
    }

    public double getB() {
        return this.b;
    }

    public Position2D getLeftRestriction() {
        return new Position2D(
                this.leftRestriction,
                this.m * this.leftRestriction + this.b,
                0
        );
    }

    public Position2D getRightRestriction() {
        return new Position2D(
                this.rightRestriction,
                this.m * this.rightRestriction + this.b,
                0
        );
    }

    public boolean inDomain(Position2D position2D) {
        return leftRestriction <= position2D.getX() && position2D.getX() <= rightRestriction;
    }
}
