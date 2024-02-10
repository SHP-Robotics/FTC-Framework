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
                        restrictedCircle.plugCircleNegative(x1) + shiftUp,
                        0
                );

                furthestIntersections[1] = new Position2D(
                        x1 + shiftRight,
                        restrictedCircle.plugCircleNegative(x1) + shiftUp,
                        0
                );

                return furthestIntersections;
            }

            return null;
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
            double approxY = m * solutionX + b;

            if ((restrictedCircle.checkInCirclePositive(solutionX) && !restrictedCircle.checkInCircleNegative(solutionX)) || (restrictedCircle.checkInCirclePositive(solutionX) && Math.abs(approxY - restrictedCircle.plugCirclePositive(solutionX) - shiftUp) < Math.abs(approxY - restrictedCircle.plugCircleNegative(solutionX) - shiftUp))) {
                furthestIntersections[0] = new Position2D(
                        solutionX + shiftRight,
                        restrictedCircle.plugCirclePositive(solutionX) + shiftUp,
                        0
                );

                if (!this.inDomain(furthestIntersections[0])) {
                    furthestIntersections[0] = null;
                }
            } else if (restrictedCircle.checkInCircleNegative(solutionX)) {
                furthestIntersections[0] = new Position2D(
                        solutionX + shiftRight,
                        restrictedCircle.plugCircleNegative(solutionX) + shiftUp,
                        0
                );

                if (!this.inDomain(furthestIntersections[0])) {
                    furthestIntersections[0] = null;
                }
            } else {
                furthestIntersections[0] = null;
            }

            furthestIntersections[1] = null;

        } else if (discriminant > 0) {
            double solutionX1 = (d * dy + sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
            double solutionX2 = (d * dy - sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);

            double approxY1 = m * solutionX1 + b;
            double approxY2 = m * solutionX2 + b;

            if ((restrictedCircle.checkInCirclePositive(solutionX1) && !restrictedCircle.checkInCircleNegative(solutionX1)) || (restrictedCircle.checkInCirclePositive(solutionX1) && Math.abs(approxY1 - restrictedCircle.plugCirclePositive(solutionX1) - shiftUp) <= Math.abs(approxY1 - restrictedCircle.plugCircleNegative(solutionX1) - shiftUp))) {
                furthestIntersections[0] = new Position2D(
                        solutionX1 + shiftRight,
                        restrictedCircle.plugCirclePositive(solutionX1) + shiftUp,
                        0
                );
            } else if (restrictedCircle.checkInCircleNegative(solutionX1)) {
                furthestIntersections[0] = new Position2D(
                        solutionX1 + shiftRight,
                        restrictedCircle.plugCircleNegative(solutionX1) + shiftUp,
                        0
                );
            } else {
                furthestIntersections[0] = null;
            }

            if ((restrictedCircle.checkInCirclePositive(solutionX2) && !restrictedCircle.checkInCircleNegative(solutionX2)) || (restrictedCircle.checkInCirclePositive(solutionX2) && Math.abs(approxY2 - restrictedCircle.plugCirclePositive(solutionX2) - shiftUp) <= Math.abs(approxY2 - restrictedCircle.plugCircleNegative(solutionX2) - shiftUp))) {
                furthestIntersections[1] = new Position2D(
                        solutionX2 + shiftRight,
                        restrictedCircle.plugCirclePositive(solutionX2) + shiftUp,
                        0
                );
            } else if (restrictedCircle.checkInCircleNegative(solutionX2)) {
                furthestIntersections[1] = new Position2D(
                        solutionX2 + shiftRight,
                        restrictedCircle.plugCircleNegative(solutionX2) + shiftUp,
                        0
                );
            } else {
                furthestIntersections[1] = null;
            }
        }

        return furthestIntersections;
    }
}
