package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedLine;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

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

    private Position2D getOptimalIntersection(Position2D currentPosition) {
        RestrictedCircle followingCircle = new RestrictedCircle(followRadius);
        followingCircle.setOffset(currentPosition);

        Position2D[] furthestIntersections = new Position2D[2];
        int waypoint = -1;

        for (int i = 0; i < this.waypoints.size()-1; i++) {
            Position2D position1 = this.waypoints.get(i).getPosition();
            Position2D position2 = this.waypoints.get(i+1).getPosition();

            double x1 = position1.getX();
            double y1 = position1.getY();

            double x2 = position2.getX();
            double y2 = position2.getY();
//
//            double x3;
//            double y3;
//
//            if (x1 == x2) {
//                x3 = x1;
//                y3 = y2 + followRadius;
//            } else {
//
//                double m = (y2 - y1) / (x2 - x1);
//                double b = y1 - (m * x1);
//
//                // double dist = Math.sqrt(x3**2 + y3**2) // = followRadius
//                // x3**2 + y3**2 = followRadius**2
//                // y3 = m*x3+b
//                // x3**2 + m**2*x3**2 + 2*m*x3*b + b**2 = followRadius **2
//                // x3**2 + x3**2*m**2 + x3*2*m*b = followRadius**2 - b**2
//                // x3**2(1 + m**2) + x3*2*m*b = followRadius**2 - b**2
//                // (1+m**2)(x3 + m*b)**2 = followRadius**2 - b**2
//                // (x3 + m*b)**2 = (followRadius**2 - b**2)/(1+m**2)
//                // x3 + m*b = +- sqrt((followRadius**2 - b**2)/(1+m**2))
//                // x3 = m*b +- sqrt((followRadius**2 - b**2)/(1+m**2))
//                //
//
//                if (x2 > x1) {
//                    x3 = m*b + Math.sqrt((followRadius*followRadius - b*b)/(1+m*m));
//                } else {
//                    x3 = m*b - Math.sqrt((followRadius*followRadius - b*b)/(1+m*m));
//                }
//                y3 = x3*m + b;
//            }

            RestrictedLine line = new RestrictedLine(x1, y1, x2, y2);

            Position2D[] tmpIntersections = followingCircle.getLineIntersections(line);
            if (currentPosition.dist(line.getP2()) <= followRadius) {
                furthestIntersections = new Position2D[]{line.getP2(), null};
                waypoint = i;
            } else if (tmpIntersections != null && tmpIntersections.length == 2 && (tmpIntersections[0] != null || tmpIntersections[1] != null)) {
                furthestIntersections = tmpIntersections;
                waypoint = i;
            }
        }

        if (furthestIntersections[0] == null) {
            return furthestIntersections[1];
        }

        if (furthestIntersections[1] == null) {
            return furthestIntersections[0];
        }

        if (waypoints.get(waypoint+1).getPosition().dist(furthestIntersections[0]) < waypoints.get(waypoint+1).getPosition().dist(furthestIntersections[1])) {
            return furthestIntersections[0];
        }

        return furthestIntersections[1];
    }

    private static double clampRadians(double radians) {
        while (radians < -Math.PI) {
            radians += (2*Math.PI);
        }
        while (radians > Math.PI) {
            radians -= (2*Math.PI);
        }
        return radians;
    }

    public void follow(MecanumPurePursuitController mecanumPurePursuitController) {
        while (!this.isFinished()) {
            this.mecanumPurePursuitController.updateOdometry();

            Position2D optimalIntersection = getOptimalIntersection(this.mecanumPurePursuitController.getCurrentPosition());

            if (optimalIntersection == null) {
                failed = true;
                return;
            }

            double differenceX = optimalIntersection.getX() - this.mecanumPurePursuitController.getCurrentPosition().getX();
            double differenceY = optimalIntersection.getY() - this.mecanumPurePursuitController.getCurrentPosition().getY();
            double differenceHeading = clampRadians(optimalIntersection.getHeadingRadians() - this.mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
            differenceHeading *= Constants.MECANUM_WIDTH;

            double max = Math.max(Math.abs(differenceX), Math.max(Math.abs(differenceY), Math.abs(differenceHeading)));

            double x = differenceX / max;
            double y = differenceY / max;
            double r = differenceHeading / max;

            this.mecanumPurePursuitController.driveFieldParams(x * this.driveSpeed, y * this.driveSpeed, r * this.driveSpeed, mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
        }
        this.mecanumPurePursuitController.deactivate();
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
            double differenceY = optimalIntersection.getY() - this.mecanumPurePursuitController.getCurrentPosition().getY();
            double differenceHeading = clampRadians(optimalIntersection.getHeadingRadians() - this.mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
            differenceHeading *= Constants.MECANUM_WIDTH;

            double max = Math.max(Math.abs(differenceX), Math.max(Math.abs(differenceY), Math.abs(differenceHeading)));

            double x = differenceX / max;
            double y = differenceY / max;
            double r = differenceHeading / max;

            this.mecanumPurePursuitController.driveFieldParams(x * this.driveSpeed, y * this.driveSpeed, r * this.driveSpeed, mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
        } else {
            this.mecanumPurePursuitController.deactivate();
            this.isFollowing = false;
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

            this.followRadius = 0.1;
            this.positionBuffer = 0.3;
            this.rotationBuffer = 0.3;

            this.driveSpeed = 0.3;
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
