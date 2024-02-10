package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

import org.apache.commons.math3.util.MathUtils;

public class RestrictedCircle extends GeometricShape {
    private final double radius;
    private double shiftRight;
    private double shiftUp;

    private Position2D endpoint;

    private double restrictionClockwiseRadians;
    private double restrictionCounterClockwiseRadians;

    public RestrictedCircle(double radius) {
        this.radius = radius;
        this.shiftRight = 0;
        this.shiftUp = 0;
    }

    public RestrictedCircle(Position2D p1, Position2D p2) {
        RestrictedLine l1 = new RestrictedLine(p1, Position2D.add(p1, new Position2D(
                Math.cos(p1.getHeadingRadians()),
                Math.sin(p1.getHeadingRadians()),
                0
        )));

        RestrictedLine l2 = new RestrictedLine(p2, Position2D.add(p2, new Position2D(
                Math.cos(p2.getHeadingRadians()),
                Math.sin(p2.getHeadingRadians()),
                0
        )));

        if (p1.getHeadingRadians() == p2.getHeadingRadians()) {

            this.radius = Math.abs(l1.getB() - l2.getB()) / Math.sqrt(1 + (l1.getM() * l1.getM()));

        } else {

            Position2D p3 = RestrictedLine.lineIntersection(l1, l2);

            double bisectorAngle = p2.getHeadingRadians() - p1.getHeadingRadians();
            double distFromIntersection = Position2D.dist(p1, p3);
            this.radius = distFromIntersection * (Math.sin(bisectorAngle / 2));

        }

        this.setOffset(new Position2D(
                p1.getX() - (this.radius * Math.cos(p1.getHeadingRadians())),
                p1.getY() - (this.radius * Math.sin(p1.getHeadingRadians())),
                0
        ));

        this.setRestrictionCounterClockwiseRadians(Math.PI - p1.getHeadingRadians());
        this.setRestrictionClockwiseRadians(Math.PI - p2.getHeadingRadians());

        this.endpoint = p2;
    }

    public double getRadius() {
        return radius;
    }

    public void setOffset(Position2D offset) {
        this.shiftRight = offset.getX();
        this.shiftUp = offset.getY();
    }

    public Position2D getOffset() {
        return new Position2D (
                this.shiftRight,
                this.shiftUp,
                0
        );
    }

    public void setRestrictionClockwiseRadians(double restrictionClockwiseRadians) {
        this.restrictionClockwiseRadians = MathUtils.normalizeAngle(restrictionClockwiseRadians, 0.0);

    }

    public void setRestrictionCounterClockwiseRadians(double restrictionCounterClockwiseRadians) {
        this.restrictionCounterClockwiseRadians = MathUtils.normalizeAngle(restrictionCounterClockwiseRadians, 0.0);
    }

    public boolean checkInCirclePositive(double x) {
        return Math.cos(Math.max(this.restrictionCounterClockwiseRadians, Math.PI)) * radius <= x && x <= Math.cos(Math.min(restrictionClockwiseRadians, Math.PI)) * radius;
    }

    public double plugCirclePositive (double x) {
        if (checkInCirclePositive(x)) {
            return Math.sqrt((radius * radius) - (x*x));
        }
        throw new IllegalArgumentException("X is not within the restriction boundaries");
    }

    public boolean checkInCircleNegative(double x) {
        return Math.cos(Math.max(restrictionClockwiseRadians, Math.PI)) * radius <= x && x <= Math.cos(Math.min(restrictionCounterClockwiseRadians, Math.PI)) * radius;
    }

    public double plugCircleNegative (double x) {
        if (checkInCircleNegative(x)) {
            return -Math.sqrt((radius * radius) - (x*x));
        }
        throw new IllegalArgumentException("X is not within the restriction boundaries");
    }

    static double sgn(double x) {
        return x < 0 ? -1: 1;
    }

    public Position2D[] getLineIntersections(RestrictedLine restrictedLine) {

        Position2D[] furthestIntersections = new Position2D[2];

        double x1 = restrictedLine.getP1().getX() - this.shiftRight;
        double y1 = restrictedLine.getP1().getY() - this.shiftUp;

        double x2 = restrictedLine.getP2().getX() - this.shiftRight;
        double y2 = restrictedLine.getP2().getY() - this.shiftUp;

        if (x1 == x2) {
            if (checkInCirclePositive(x1)) {
                furthestIntersections[0] = new Position2D(
                        x1 + this.shiftRight,
                        plugCirclePositive(x1) + this.shiftUp,
                        0
                );

                furthestIntersections[1] = new Position2D(
                        x1 + this.shiftRight,
                        plugCircleNegative(x1) + this.shiftUp,
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

        double discriminant = this.radius * this.radius * dr * dr - d * d;

        if (discriminant == 0) {
            double solutionX = (d*dy) / (dr*dr);
            double approxY = m * solutionX + b;

            if ((checkInCirclePositive(solutionX) && !checkInCircleNegative(solutionX)) || (checkInCirclePositive(solutionX) && Math.abs(approxY - plugCirclePositive(solutionX) - this.shiftUp) < Math.abs(approxY - plugCircleNegative(solutionX) - this.shiftUp))) {
                furthestIntersections[0] = new Position2D(
                        solutionX + this.shiftRight,
                        plugCirclePositive(solutionX) + this.shiftUp,
                        0
                );

                if (!restrictedLine.inDomain(furthestIntersections[0])) {
                    furthestIntersections[0] = null;
                }
            } else if (checkInCircleNegative(solutionX)) {
                furthestIntersections[0] = new Position2D(
                        solutionX + this.shiftRight,
                        plugCircleNegative(solutionX) + this.shiftUp,
                        0
                );

                if (!restrictedLine.inDomain(furthestIntersections[0])) {
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

            if ((checkInCirclePositive(solutionX1) && !checkInCircleNegative(solutionX1)) || (checkInCirclePositive(solutionX1) && Math.abs(approxY1 - plugCirclePositive(solutionX1) - this.shiftUp) <= Math.abs(approxY1 - plugCircleNegative(solutionX1) - this.shiftUp))) {
                furthestIntersections[0] = new Position2D(
                        solutionX1 + this.shiftRight,
                        plugCirclePositive(solutionX1) + this.shiftUp,
                        0
                );
            } else if (checkInCircleNegative(solutionX1)) {
                furthestIntersections[0] = new Position2D(
                        solutionX1 + this.shiftRight,
                        plugCircleNegative(solutionX1) + this.shiftUp,
                        0
                );
            } else {
                furthestIntersections[0] = null;
            }

            if ((checkInCirclePositive(solutionX2) && !checkInCircleNegative(solutionX2)) || (checkInCirclePositive(solutionX2) && Math.abs(approxY2 - plugCirclePositive(solutionX2) - this.shiftUp) <= Math.abs(approxY2 - plugCircleNegative(solutionX2) - this.shiftUp))) {
                furthestIntersections[1] = new Position2D(
                        solutionX2 + this.shiftRight,
                        plugCirclePositive(solutionX2) + this.shiftUp,
                        0
                );
            } else if (checkInCircleNegative(solutionX2)) {
                furthestIntersections[1] = new Position2D(
                        solutionX2 + this.shiftRight,
                        plugCircleNegative(solutionX2) + this.shiftUp,
                        0
                );
            } else {
                furthestIntersections[1] = null;
            }
        }

        return furthestIntersections;
    }

    public Position2D[] circleIntersections(RestrictedCircle restrictedCircle) {
        Position2D circleOffset = restrictedCircle.getOffset();
        circleOffset.add(new Position2D(
                -this.shiftRight,
                -this.shiftUp,
                0
        ), true);

        circleOffset.rotate(-Math.atan(circleOffset.getY() / circleOffset.getX()));

        if (circleOffset.getX() == 0) {
            return new Position2D[]{
                    null, null
            };
        }

        double solutionX = ((circleOffset.getX()*circleOffset.getX())
                - (this.radius * this.radius)
                + (restrictedCircle.radius * restrictedCircle.radius))
                / (2 * circleOffset.getX());

        if (checkInCirclePositive(solutionX)) {
            double solutionY1 = plugCirclePositive(solutionX);

            if (checkInCircleNegative(solutionX)) {
                double solutionY2 = plugCircleNegative(solutionX);

                if (solutionY1 == solutionY2) {
                    return new Position2D[]{new Position2D(
                            solutionX + this.shiftRight,
                            solutionY1 + this.shiftUp,
                            0
                    ), null};
                }

                return new Position2D[]{
                        new Position2D(
                                solutionX + this.shiftRight,
                                solutionY1 + this.shiftUp,
                                0
                        ), new Position2D(
                        solutionX + this.shiftRight,
                        solutionY2 + this.shiftUp,
                        0
                )};
            }

            return new Position2D[]{new Position2D(
                    solutionX + this.shiftRight,
                    solutionY1 + this.shiftUp,
                    0
            ), null};
        }

        if (checkInCircleNegative(solutionX)) {
            return new Position2D[]{new Position2D(
                    solutionX + this.shiftRight,
                    plugCircleNegative(solutionX) + this.shiftUp,
                    0
            ), null};
        }

        return new Position2D[]{null,null};
    }

    public Position2D getEndpoint() {
        return this.endpoint;
    }
}
