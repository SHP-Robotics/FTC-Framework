package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints.Waypoint;

import java.util.ArrayList;

public class PurePursuitPath {
    private final ArrayList<Waypoint> waypoints;

    private final double followRadius;
    private final double positionBuffer;
    private final double rotationBuffer;

    private final double driveSpeed;

    private boolean isFollowing = false;
    private boolean failed = false;
    private MecanumPurePursuitController mecanumPurePursuitController;

    public PurePursuitPath(PurePursuitPathBuilder purePursuitPathBuilder) {
        this.waypoints = purePursuitPathBuilder.waypoints;

        this.followRadius = purePursuitPathBuilder.followRadius;
        this.positionBuffer = purePursuitPathBuilder.positionBuffer;
        this.rotationBuffer = purePursuitPathBuilder.rotationBuffer;

        this.driveSpeed = purePursuitPathBuilder.driveSpeed;
    }

    private static double plugCirclePositive (
            double radius,
            double x,
            double shiftRight,
            double shiftUp) {
        return Math.sqrt((radius * radius) - ((x - shiftRight) * (x - shiftRight))) + shiftUp;
    }

    private static double plugCircleNegative (
            double radius,
            double x,
            double shiftRight,
            double shiftUp) {
        return shiftUp - Math.sqrt((radius * radius) - ((x - shiftRight) * (x - shiftRight)));
    }

    private static double solvePositive(
            double radius,
            double shiftRight,
            double shiftUp,
            double slope,
            double offset
    ) {
        return (Math.sqrt(
                -offset*offset + 2 * offset * shiftUp - 2 * offset * shiftRight * slope - shiftUp * shiftUp + 2 * shiftUp * shiftRight * slope - shiftRight * shiftRight * slope * slope + slope * slope * radius * radius + radius * radius
        ) - offset * slope + shiftUp * slope + shiftRight)
                / ((slope*slope) +1);
    }

    private static double solveNegative(
            double radius,
            double shiftRight,
            double shiftUp,
            double slope,
            double offset
    ) {
        return (-Math.sqrt(
                -offset*offset + 2 * offset * shiftUp - 2 * offset * shiftRight * slope - shiftUp * shiftUp + 2 * shiftUp * shiftRight * slope - shiftRight * shiftRight * slope * slope + slope * slope * radius * radius + radius * radius
        ) - offset * slope + shiftUp * slope + shiftRight)
                / ((slope*slope) +1);
    }

    private static boolean checkSolve(
            double radius,
            double shiftRight,
            double shiftUp,
            double slope,
            double offset
    ) {
        return (-offset*offset + 2 * offset * shiftUp - 2 * offset * shiftRight * slope - shiftUp * shiftUp + 2 * shiftUp * shiftRight * slope - shiftRight * shiftRight * slope * slope + slope * slope * radius * radius + radius * radius) >= 0;
    }

    private Position2D getOptimalIntersection(Position2D currentPosition) {
        Position2D[] furthestIntersections = new Position2D[2];
        int waypoint = -1;

        for (int i = 0; i < this.waypoints.size()-1; i++) {
            Position2D position1 = this.waypoints.get(i).getPosition();
            Position2D position2 = this.waypoints.get(i+1).getPosition();


            double x1 = position1.getX();
            double y1 = position1.getY();

            double x2 = position2.getX();
            double y2 = position2.getY();

            if (x1 == x2) {
                waypoint = i;

                if (Math.abs(currentPosition.getX() - x1) <= this.followRadius) {
                    furthestIntersections[0] = new Position2D(x1,
                                    plugCirclePositive(this.followRadius, x1, currentPosition.getX(), currentPosition.getY()),
                                    0);

                    furthestIntersections[1] = new Position2D(x1,
                            plugCircleNegative(this.followRadius, x1, currentPosition.getX(), currentPosition.getY()),
                            0);
                }

                continue;
            }

            double m = (y2 - y1) / (x2 - x1);
            double b = y1 - m*x1;

            if (checkSolve(
                    this.followRadius,
                    currentPosition.getX(),
                    currentPosition.getY(),
                    m,
                    b
            )) {
                waypoint = i;

                double solutionX1 = solvePositive(
                        this.followRadius,
                        currentPosition.getX(),
                        currentPosition.getY(),
                        m,
                        b
                );

                double solutionX2 = solveNegative(
                        this.followRadius,
                        currentPosition.getX(),
                        currentPosition.getY(),
                        m,
                        b
                );

                double solutionY11 = plugCirclePositive(
                        this.followRadius,
                        solutionX1,
                        currentPosition.getX(),
                        currentPosition.getY()
                );

                double solutionY12 = plugCircleNegative(
                        this.followRadius,
                        solutionX1,
                        currentPosition.getX(),
                        currentPosition.getY()
                );

                double solutionY21 = plugCirclePositive(
                        this.followRadius,
                        solutionX2,
                        currentPosition.getX(),
                        currentPosition.getY()
                );

                double solutionY22 = plugCircleNegative(
                        this.followRadius,
                        solutionX2,
                        currentPosition.getX(),
                        currentPosition.getY()
                );

                if (Math.abs((m * solutionX1) - solutionY11) < Math.abs((m * solutionX1) - solutionY12)) {
                    furthestIntersections[0] = new Position2D(
                            solutionX1,
                            solutionY11,
                            0
                    );
                } else {
                    furthestIntersections[0] = new Position2D(
                            solutionX1,
                            solutionY12,
                            0
                    );
                }

                if (Math.abs((m * solutionX2) - solutionY21) < Math.abs((m * solutionX2) - solutionY22)) {
                    furthestIntersections[1] = new Position2D(
                            solutionX2,
                            solutionY21,
                            0
                    );
                } else {
                    furthestIntersections[1] = new Position2D(
                            solutionX2,
                            solutionY22,
                            0
                    );
                }
            }
        }

        if (furthestIntersections[0] == null) {
            return null;
        }

        if (furthestIntersections[1] == null) {
            return furthestIntersections[0];
        }

        if (this.waypoints.get(waypoint).getPosition().dist(furthestIntersections[0]) < this.waypoints.get(waypoint).getPosition().dist(furthestIntersections[1])) {
            return furthestIntersections[0];
        }

        return furthestIntersections[1];
    }

    public void follow(MecanumPurePursuitController mecanumPurePursuitController) {
        while (!this.isFinished()) {
            mecanumPurePursuitController.updateOdometry();

            Position2D optimalIntersection = getOptimalIntersection(mecanumPurePursuitController.getCurrentPosition());

            if (optimalIntersection == null) {
                failed = true;
                return;
            }

            double differenceX = optimalIntersection.getX() - mecanumPurePursuitController.getCurrentPosition().getX();
            double differenceY = optimalIntersection.getX() - mecanumPurePursuitController.getCurrentPosition().getX();
            double differenceHeading = optimalIntersection.getX() - mecanumPurePursuitController.getCurrentPosition().getX();
            differenceHeading *= mecanumPurePursuitController.getMecanumWidth();

            double max = Math.max(differenceX, Math.max(differenceY, differenceHeading));

            double x = differenceX / max;
            double y = differenceY / max;
            double r = differenceHeading / max;

            mecanumPurePursuitController.driveParams(x*this.driveSpeed, y*this.driveSpeed, r*this.driveSpeed);
        }
    }

    public void followAsync(MecanumPurePursuitController mecanumPurePursuitController) {
        this.isFollowing = true;
        this.mecanumPurePursuitController = mecanumPurePursuitController;
    }

    public void update() {
        if (!this.isFinished()) {
            this.mecanumPurePursuitController.updateOdometry();

            Position2D optimalIntersection = getOptimalIntersection(this.mecanumPurePursuitController.getCurrentPosition());

            if (optimalIntersection == null) {
                failed = true;
                return;
            }

            double differenceX = optimalIntersection.getX() - this.mecanumPurePursuitController.getCurrentPosition().getX();
            double differenceY = optimalIntersection.getX() - this.mecanumPurePursuitController.getCurrentPosition().getX();
            double differenceHeading = optimalIntersection.getX() - this.mecanumPurePursuitController.getCurrentPosition().getX();
            differenceHeading *= this.mecanumPurePursuitController.getMecanumWidth();

            double max = Math.max(differenceX, Math.max(differenceY, differenceHeading));

            double x = differenceX / max;
            double y = differenceY / max;
            double r = differenceHeading / max;

            this.mecanumPurePursuitController.driveParams(x * this.driveSpeed, y * this.driveSpeed, r * this.driveSpeed);
        } else {
            this.isFollowing = false;
            this.mecanumPurePursuitController = null;
        }
    }

    public boolean isFinished() {
        if (!isFollowing) {
            Position2D lastPosition = this.waypoints.get(this.waypoints.size()-1).getPosition();
            return this.failed() ||
                    (this.mecanumPurePursuitController.getCurrentPosition().dist(lastPosition) <= this.positionBuffer
                    && Math.abs(this.mecanumPurePursuitController.getCurrentPosition().getHeadingRadians() - lastPosition.getHeadingRadians()) <= rotationBuffer);
        }
        return false;
    }

    public boolean failed() {
        return failed;
    }

    public void reset() {
        this.isFollowing = false;
        this.mecanumPurePursuitController = null;
    }

    public static class PurePursuitPathBuilder {
        private ArrayList<Waypoint> waypoints;

        private double followRadius;
        private double positionBuffer;
        private double rotationBuffer;

        private double driveSpeed;

        public PurePursuitPathBuilder(StartWaypoint startWaypoint) {
            waypoints = new ArrayList<>();
            waypoints.add(startWaypoint);
        }

        public PurePursuitPathBuilder addWaypoint(Waypoint waypoint) {
            waypoints.add(waypoint);
            return this;
        }

        public PurePursuitPathBuilder setFollowRadius(double followRadius) {
            this.followRadius = followRadius;
            return this;
        }

        public PurePursuitPathBuilder setPositionBuffer(double positionBuffer) {
            this.positionBuffer = positionBuffer;
            return this;
        }

        public PurePursuitPathBuilder setRotationBuffer(double rotationBuffer) {
            this.rotationBuffer = rotationBuffer;
            return this;
        }

        public PurePursuitPathBuilder setDriveSpeed(double driveSpeed) {
            this.driveSpeed = driveSpeed;
            return this;
        }

        private void compile() {
            if (waypoints.size() < 2) {
                throw new IllegalArgumentException("Pure Pursuit Paths must start with a StartWaypoint and end with an EndWaypoint");
            }

            if (!(waypoints.get(0) instanceof StartWaypoint)) {
                throw new IllegalArgumentException("Pure Pursuit Paths must start with a StartWaypoint");
            }

            if (!(waypoints.get(waypoints.size()-1) instanceof EndWaypoint)) {
                throw new IllegalArgumentException("Pure Pursuit Paths must end with an EndWaypoint");
            }
        }

        public PurePursuitPath build() {
            compile();

            return new PurePursuitPath(this);
        }
    }
}
