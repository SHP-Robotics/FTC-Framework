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

    public static Position2D lineIntersection(RestrictedLine line1, RestrictedLine line2) {
        double m1 = (line1.p2.getY() - line1.p1.getY()) / (line1.p2.getX() - line1.p1.getX());
        double b1 = line1.p1.getY() - (m1 * line1.p1.getX());

        double m2 = (line2.getP2().getY() - line2.getP1().getY()) / (line2.getP2().getX() - line2.getP1().getX());
        double b2 = line2.getP1().getY() - (m1 * line2.getP1().getX());

        // m1*x + b1 = m2*x + b2
        // (m1-m2)*x = b2-b1

        // solutionX = (b2-b1)/(m1-m2)
        // solutionY = m1*(b2-b1)/(m1-m2) + b1

        return new Position2D(
                (b2-b1)/(m1-m2),
                m1*((b2-b1)/(m1-m2))+b1,
                0
        );
    }
}
