package org.firstinspires.ftc.teamcode.PurePursuit.Geometry;

import org.apache.commons.math3.util.MathUtils;

public class RestrictedCircle extends GeometricShape {
    private final double radius;
    private double shiftRight;
    private double shiftUp;
    private double rotation;

    private Position2D endpoint;

    private double restrictionClockwiseRadians;
    private double restrictionCounterClockwiseRadians;

    private final boolean rotating;

    public RestrictedCircle(double radius) {
        this.radius = radius;
        this.shiftRight = 0;
        this.shiftUp = 0;
        this.rotation = 0;
        this.rotating = false;
    }

    public RestrictedCircle(Position2D p1, Position2D p2) {
        RestrictedLine l1 = new RestrictedLine(p1, Position2D.add(p1, new Position2D(
                Math.cos(p1.getHeadingRadians()),
                Math.sin(p1.getHeadingRadians()),
                p1.getHeadingRadians()
        )));

        RestrictedLine l2 = new RestrictedLine(p2, Position2D.add(p2, new Position2D(
                Math.cos(p2.getHeadingRadians()),
                Math.sin(p2.getHeadingRadians()),
                p2.getHeadingRadians()
        )));

        if (p1.getHeadingRadians() == p2.getHeadingRadians()) {
            throw new IllegalArgumentException("headings between point1 and point2 cannot be equal");
        }

        if (p1.getHeadingRadians() == -p2.getHeadingRadians()) {

            if (Math.abs(p1.getHeadingRadians()) == Math.PI/2) {
                this.radius = Math.abs((l1.getP1().getX() - l2.getP1().getX()) / 2);
            } else {
                this.radius = Math.abs(l1.getB() - l2.getB()) / (2 * Math.sqrt(1 + (l1.getM() * l1.getM())));
            }

        } else {

            Position2D p3 = RestrictedLine.lineIntersection(l1, l2);

            double bisectorAngle = p2.getHeadingRadians() - p1.getHeadingRadians();
            double distFromIntersection = Position2D.dist(p1, p3);
            this.radius = distFromIntersection * (Math.sin(bisectorAngle / 2));

        }

        this.setOffset(new Position2D(
                Math.min(p1.getX(), p2.getX()) + (this.radius * Math.sin(p1.getHeadingRadians())),
                Math.min(p1.getY(), p2.getY()) + (this.radius * Math.cos(p1.getHeadingRadians())),
                0
        ));

        this.setRestrictionCounterClockwiseRadians((Math.PI/2) - p1.getHeadingRadians());
        this.setRestrictionClockwiseRadians((Math.PI/2) - p2.getHeadingRadians());

        this.endpoint = p2;
        this.rotating = true;
    }

    public double getRadius() {
        return radius;
    }

    public void setOffset(Position2D offset) {
        this.shiftRight = offset.getX();
        this.shiftUp = offset.getY();
        this.rotation = offset.getHeadingRadians();
    }

    public Position2D getOffset() {
        return new Position2D (
                this.shiftRight,
                this.shiftUp,
                this.rotation
        );
    }

    public void setRestrictionClockwiseRadians(double restrictionClockwiseRadians) {
        this.restrictionClockwiseRadians = MathUtils.normalizeAngle(restrictionClockwiseRadians, 0.0);

    }

    public void setRestrictionCounterClockwiseRadians(double restrictionCounterClockwiseRadians) {
        this.restrictionCounterClockwiseRadians = MathUtils.normalizeAngle(restrictionCounterClockwiseRadians, 0.0);
    }

    public boolean checkInCirclePositive(double x) {
        return Math.cos(Math.max(this.restrictionClockwiseRadians, Math.PI)) * radius <= x && x <= Math.cos(Math.min(restrictionCounterClockwiseRadians, Math.PI)) * radius;
    }

    public double plugCirclePositive (double x) {
        return Math.sqrt((radius * radius) - (x*x));
    }

    public boolean checkInCircleNegative(double x) {
        return Math.cos(Math.max(restrictionCounterClockwiseRadians, Math.PI)) * radius <= x && x <= Math.cos(Math.min(restrictionClockwiseRadians, Math.PI)) * radius;
    }

    public double plugCircleNegative (double x) {
        return -Math.sqrt((radius * radius) - (x*x));
    }

    public boolean checkInCircle(double x, double y) {
        if (Math.abs(y - plugCirclePositive(x)) < Math.abs(y - plugCircleNegative(x))) {
            return checkInCirclePositive(x);
        }
        return checkInCircleNegative(x);
    }

    public boolean checkInCircle(Position2D position2D) {
        double x = position2D.getX();
        double y = position2D.getY();
        return this.checkInCircle(x, y);
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

//    public double getHeading(Position2D position2D) {
//        double optimalHeading;
//
//        if (!this.rotating) {
//            optimalHeading = Math.atan(position2D.getY()/position2D.getX());
//        } else {
//            optimalHeading = (Math.PI / 2) + Math.atan(position2D.getY() / position2D.getX());
//        }
//
//        return optimalHeading;
//    }

    public double getIntersectionHeading(Position2D position2D) {
        return Math.atan(position2D.getY()/position2D.getX());
    }

    public Position2D[] circleIntersections(RestrictedCircle restrictedCircle) {
        Position2D circleOffset = restrictedCircle.getOffset();
        circleOffset.add(new Position2D(
                -this.shiftRight,
                -this.shiftUp,
                0
        ), true);

//        double heading = this.getHeading(circleOffset);

        double rotationRadians = Math.atan(circleOffset.getY() / circleOffset.getX());

        circleOffset.rotate(-rotationRadians);

        if (circleOffset.getX() == 0) {
            return new Position2D[]{
                    null, null
            };
        }

        double solutionX = ((circleOffset.getX()*circleOffset.getX())
                - (restrictedCircle.radius * restrictedCircle.radius)
                + (this.radius * this.radius))
                / (2 * circleOffset.getX());

        double solutionY1 = plugCirclePositive(solutionX);
        double solutionY2 = -solutionY1;

        Position2D solution1 = new Position2D(solutionX, solutionY1, 0);
        solution1.rotate(rotationRadians);
        solution1.setHeadingRadians(getIntersectionHeading(Position2D.add(solution1, circleOffset.getNegative())), true);

        Position2D solution2 = new Position2D(solutionX, solutionY2, 0);
        solution2.rotate(rotationRadians);
        solution2.setHeadingRadians(getIntersectionHeading(Position2D.add(solution2, circleOffset.getNegative())), true);

        if (!checkInCircle(solution1)) {
            solution1 = null;
        } else {
            solution1.add(this.getOffset(), true);
        }

        if (!checkInCircle(solution2)) {
            solution2 = null;
        } else {
            solution2.add(this.getOffset(), true);
        }

        return new Position2D[]{solution1, solution2};
    }

    @Override
    public Position2D getEndpoint() {
        return this.endpoint;
    }

    public Position2D getEndpoint(RestrictedCircle followingCircle) {
        double percentClose = followingCircle.getOffset().dist(this.getEndpoint()) / followingCircle.getRadius();
        double dist = (this.endpoint.getHeadingRadians()-followingCircle.getOffset().getHeadingRadians());

        return new Position2D(
                this.endpoint.getX(),
                this.endpoint.getY(),
                (dist * percentClose) + followingCircle.getOffset().getHeadingRadians()
        );
    }
}
