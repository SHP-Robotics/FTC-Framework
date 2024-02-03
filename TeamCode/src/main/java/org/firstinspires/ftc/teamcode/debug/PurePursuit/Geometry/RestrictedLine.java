package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public class RestrictedLine {
    private Position2D p1;
    private Position2D p2;

    public RestrictedLine(Position2D p1, Position2D p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    public RestrictedLine(double x1, double y1, double x2, double y2) {
        this.p1 = new Position2D(x1, y1, 0);
        this.p2 = new Position2D(x2, y2, 0);
    }

    public Position2D getP1() {
        return this.p1;
    }

    public Position2D getP2() {
        return this.p2;
    }

    public boolean inDomain(Position2D position2D) {
        return p1.getX() <= position2D.getX() && position2D.getX() <= p2.getX();
    }
}
