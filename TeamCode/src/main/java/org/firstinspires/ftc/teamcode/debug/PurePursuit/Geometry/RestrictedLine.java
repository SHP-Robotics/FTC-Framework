package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public class RestrictedLine extends GeometricShape {
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

    public Position2D getEndpoint() {
        return this.getP2();
    }

    public double getM() {
        return (this.p2.getY() - this.p1.getY()) / (this.p2.getX() - this.p1.getX());
    }

    public double getB() {
        return this.p1.getY() - (this.getM() * this.p1.getX());
    }

    public boolean inDomain(Position2D position2D) {
        return p1.getX() <= position2D.getX() && position2D.getX() <= p2.getX();
    }

    static double sgn(double x) {
        return x < 0 ? -1: 1;
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

    public Position2D[] circleIntersections(RestrictedCircle restrictedCircle) {

        Position2D[] furthestIntersections = new Position2D[2];

        double shiftRight = restrictedCircle.getOffset().getX();
        double shiftUp = restrictedCircle.getOffset().getY();

        double x1 = this.getP1().getX() - shiftRight;
        double y1 = this.getP1().getY() - shiftUp;

        double x2 = this.getP2().getX() - shiftRight;
        double y2 = this.getP2().getY() - shiftUp;

        if (x1 == x2) {
            if (restrictedCircle.checkInCirclePositive(x1)) {
                furthestIntersections[0] = new Position2D(
                        x1 + shiftRight,
                        restrictedCircle.plugCirclePositive(x1) + shiftUp,
                        0
                );
            }

            if (restrictedCircle.checkInCircleNegative(x1)) {
                furthestIntersections[1] = new Position2D(
                        x1 + shiftRight,
                        restrictedCircle.plugCircleNegative(x1) + shiftUp,
                        0
                );
            }

            return furthestIntersections;
        }

        double m = (y2-y1)/(x2-x1);
        double b = y1 - (m*x1);

        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = Math.sqrt(dx * dx + dy * dy);
        double d = x1 * y2 - x2 * y1;

        double discriminant = restrictedCircle.getRadius() * restrictedCircle.getRadius() * dr * dr - d * d;

        if (discriminant == 0) {
            double solutionX = (d*dy) / (dr*dr);
            double solutionY = (-d*dx) / (dr*dr);

            furthestIntersections[0] = new Position2D(
                    solutionX,
                    solutionY,
                    0
            );

            furthestIntersections[1] = null;

        } else if (discriminant > 0) {
            double solutionX1 = (d * dy + sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
            double solutionX2 = (d * dy - sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);

            double solutionY1 = (-d*dx + Math.abs(dy)*Math.sqrt(discriminant)) / (dr*dr);
            double solutionY2 = (-d*dx - Math.abs(dy)*Math.sqrt(discriminant)) / (dr*dr);

            furthestIntersections[0] = new Position2D(
                    solutionX1 + shiftRight,
                    solutionY1 + shiftUp,
                    0
            );

            furthestIntersections[1] = new Position2D(
                    solutionX2 + shiftRight,
                    solutionY2 + shiftUp,
                    0
            );
        }

        return furthestIntersections;
    }
}
