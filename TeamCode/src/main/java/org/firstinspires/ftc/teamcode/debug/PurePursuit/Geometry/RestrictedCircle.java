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

    private static double clampRadians(double x) {
        while (x < 0) {
            x += (2*Math.PI);
        }

        while (x > 2*Math.PI) {
            x -= (2*Math.PI);
        }

        return x;
    }

    public void setRestrictionClockwiseRadians(double restrictionClockwiseRadians) {
        this.restrictionClockwiseRadians = clampRadians(restrictionClockwiseRadians);

    }

    public void setRestrictionCounterClockwiseRadians(double restrictionCounterClockwiseRadians) {
        this.restrictionCounterClockwiseRadians = clampRadians(restrictionCounterClockwiseRadians);
    }

    private static double sgn(double x) {
        return x < 0 ? -1: 1;
    }

    private boolean checkInCirclePositive(double x) {
        return Math.cos(Math.max(this.restrictionCounterClockwiseRadians, Math.PI)) * radius <= x && x <= Math.cos(Math.min(restrictionClockwiseRadians, Math.PI)) * radius;
    }

    private double plugCirclePositive (double x) {
        if (checkInCirclePositive(x)) {
            return Math.sqrt((radius * radius) - (x*x));
        }
        throw new IllegalArgumentException("X is not within the restriction boundaries");
    }

    private boolean checkInCircleNegative(double x) {
        return Math.cos(Math.max(restrictionClockwiseRadians, Math.PI)) * radius <= x && x <= Math.cos(Math.min(restrictionCounterClockwiseRadians, Math.PI)) * radius;
    }

    private double plugCircleNegative (double x) {
        if (checkInCircleNegative(x)) {
            return -Math.sqrt((radius * radius) - (x*x));
        }
        throw new IllegalArgumentException("X is not within the restriction boundaries");
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
}
