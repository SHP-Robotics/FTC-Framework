package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public class RestrictedCircle {
    private final double radius;
    private double shiftRight;
    private double shiftUp;

    // TODO: implement use for restriction in intersections and plugging in
    private double restrictionClockwiseRadians;
    private double restrictionCounterClockwiseRadians;

    public RestrictedCircle(double radius) {
        this.radius = radius;
        this.shiftRight = 0;
        this.shiftUp = 0;
    }

    public void setOffset(Position2D offset) {
        this.shiftRight = offset.getX();
        this.shiftUp = offset.getY();
    }

    public void setRestrictionClockwiseRadians(double restrictionClockwiseRadians) {
        this.restrictionClockwiseRadians = restrictionClockwiseRadians;
    }

    public void setRestrictionCounterClockwiseRadians(double restrictionCounterClockwiseRadians) {
        this.restrictionCounterClockwiseRadians = restrictionCounterClockwiseRadians;
    }

    private static double sgn(double x) {
        return x < 0 ? -1: 1;
    }

    private boolean checkInCirclePositive(double x) {
        return Math.cos(2*Math.PI - Math.max(restrictionClockwiseRadians, Math.PI)) * radius + shiftRight <= x && x <= shiftRight - Math.cos(Math.PI - Math.min(restrictionCounterClockwiseRadians, Math.PI)) * radius;
    }

    private double plugCirclePositive (double x) {
        if (checkInCirclePositive(x)) {
            return Math.sqrt((radius * radius) - ((x - shiftRight) * (x - shiftRight))) + shiftUp;
        }
        throw new IllegalArgumentException("X is not within the restriction boundaries");
    }

    private boolean checkInCircleNegative(double x) {
        return Math.cos(Math.max(restrictionCounterClockwiseRadians, Math.PI)) * radius + shiftRight <= x && x <= Math.cos(Math.min(restrictionClockwiseRadians, Math.PI)) * radius + shiftRight;
    }

    private double plugCircleNegative (double x) {
        if (checkInCircleNegative(x)) {
            return -Math.sqrt((radius * radius) - ((x - shiftRight) * (x - shiftRight))) + shiftUp;
        }
        throw new IllegalArgumentException("X is not within the restriction boundaries");
    }

    public Position2D[] getLineIntersections(RestrictedLine restrictedLine) {
        Position2D[] furthestIntersections = new Position2D[2];

        double x1 = restrictedLine.getP1().getX();
        double y1 = restrictedLine.getP1().getY();

        double x2 = restrictedLine.getP2().getX();
        double y2 = restrictedLine.getP2().getY();

        if (x1 == x2) {
            if (checkInCirclePositive(x1-this.shiftRight)) {
                furthestIntersections[0] = new Position2D(
                        x1-this.shiftRight,
                        plugCirclePositive(x1-this.shiftRight),
                        0
                );

                furthestIntersections[1] = new Position2D(
                        x1-this.shiftRight,
                        plugCircleNegative(x1-this.shiftRight),
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
            double solutionX = d / dr;
            double approxY = m * (solutionX - this.shiftRight) + b;

            if ((checkInCirclePositive(solutionX) && !checkInCircleNegative(solutionX)) || (checkInCirclePositive(solutionX) && Math.abs(approxY - plugCirclePositive(solutionX)) < Math.abs(approxY - plugCircleNegative(solutionX)))) {
                furthestIntersections[0] = new Position2D(
                        solutionX - this.shiftRight,
                        plugCirclePositive(solutionX),
                        0
                );

                if (!restrictedLine.inDomain(furthestIntersections[0])) {
                    furthestIntersections[0] = null;
                }
            } else if (checkInCircleNegative(solutionX)) {
                furthestIntersections[0] = new Position2D(
                        solutionX - this.shiftRight,
                        plugCircleNegative(solutionX),
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
            double solutionX1 = (d * dr + sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);
            double solutionX2 = (d * dr - sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr);

            double approxY1 = m * (solutionX1 - this.shiftRight) + b;
            double approxY2 = m * (solutionX2 - this.shiftRight) + b;

            if ((checkInCirclePositive(solutionX1) && !checkInCircleNegative(solutionX1)) || (checkInCirclePositive(solutionX1) && Math.abs(approxY1 - plugCirclePositive(solutionX1)) < Math.abs(approxY1 - plugCircleNegative(solutionX1)))) {
                furthestIntersections[0] = new Position2D(
                        solutionX1 - this.shiftRight,
                        plugCirclePositive(solutionX1),
                        0
                );
            } else if (checkInCircleNegative(solutionX1)) {
                furthestIntersections[0] = new Position2D(
                        solutionX1 - this.shiftRight,
                        plugCircleNegative(solutionX1),
                        0
                );
            } else {
                furthestIntersections[0] = null;
            }

            if ((checkInCirclePositive(solutionX1) && !checkInCircleNegative(solutionX1)) || (checkInCirclePositive(solutionX1) && Math.abs(approxY2 - plugCirclePositive(solutionX1)) < Math.abs(approxY2 - plugCircleNegative(solutionX1)))) {
                furthestIntersections[1] = new Position2D(
                        solutionX2 - this.shiftRight,
                        plugCirclePositive(solutionX2),
                        0
                );
            } else if (checkInCircleNegative(solutionX1)) {
                furthestIntersections[1] = new Position2D(
                        solutionX2 - this.shiftRight,
                        plugCircleNegative(solutionX2),
                        0
                );
            } else {
                furthestIntersections[1] = null;
            }
        }

        return furthestIntersections;
    }
}
