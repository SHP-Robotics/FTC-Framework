package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedLine;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.InterruptionWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.LineBindingWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

import java.util.ArrayList;

public class PurePursuitPath {
    private final ArrayList<Waypoint> waypoints;

    private final double followRadius;
    private final double positionBuffer;
    private final double rotationBuffer;

    private final double tanhPace;
    private final double minimumTanh;
    private final double maximumTanh;

    private boolean isFollowing = false;
    private boolean failed = false;
    private MecanumPurePursuitController mecanumPurePursuitController;

    public PurePursuitPath(PurePursuitPathBuilder purePursuitPathBuilder) {
        this.waypoints = purePursuitPathBuilder.waypoints;

        this.followRadius = purePursuitPathBuilder.followRadius;
        this.positionBuffer = purePursuitPathBuilder.positionBuffer;
        this.rotationBuffer = purePursuitPathBuilder.rotationBuffer;

        this.tanhPace = purePursuitPathBuilder.tanhPace;
        this.minimumTanh = purePursuitPathBuilder.minimumTanh;
        this.maximumTanh = purePursuitPathBuilder.maximumTanh;
    }

    private Position2D getOptimalIntersection(Position2D currentPosition) {
        RestrictedCircle followingCircle = new RestrictedCircle(followRadius);
        followingCircle.setOffset(currentPosition);

        Position2D[] furthestIntersections = new Position2D[2];
        int waypoint = -1;

        for (int i = 0; i < this.waypoints.size()-1; i++) {
            if (this.waypoints.get(i) instanceof InterruptionWaypoint && !((InterruptionWaypoint) this.waypoints.get(i)).isExcecuted() && nearPosition(this.waypoints.get(i).getEndpoint())) {
                ((InterruptionWaypoint) this.waypoints.get(i)).run();
            }

            Position2D position1 = this.waypoints.get(i).getEndpoint();
            Position2D position2 = this.waypoints.get(i + 1).getEndpoint();

            if (this.waypoints.get(i+1) instanceof LineBindingWaypoint) {
                RestrictedCircle restrictedCircle = new RestrictedCircle(position1, position2);

                Position2D[] tmpIntersections = followingCircle.getCircleIntersections(restrictedCircle);

                if (currentPosition.dist(restrictedCircle.getEndpoint()) <= followRadius) {
                    furthestIntersections = new Position2D[]{restrictedCircle.getEndpoint(), null};
                    waypoint = i;
                } else if (tmpIntersections != null && tmpIntersections.length == 2 && (tmpIntersections[0] != null || tmpIntersections[1] != null)) {
                    furthestIntersections = tmpIntersections;
                    waypoint = i;
                }
            } else {

                double x1 = position1.getX();
                double y1 = position1.getY();

                double x2 = position2.getX();
                double y2 = position2.getY();

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
        }

        if (furthestIntersections[0] == null) {
            return furthestIntersections[1];
        }

        if (furthestIntersections[1] == null) {
            return furthestIntersections[0];
        }

        if (waypoints.get(waypoint+1).getEndpoint().dist(furthestIntersections[0]) < waypoints.get(waypoint+1).getEndpoint().dist(furthestIntersections[1])) {
            return furthestIntersections[0];
        }

        return furthestIntersections[1];
    }

    public void follow(MecanumPurePursuitController mecanumPurePursuitController) {
        this.isFollowing = true;
        this.mecanumPurePursuitController = mecanumPurePursuitController;
        while (!this.isFinished()) {
            update();
        }
        if (mecanumPurePursuitController != null) {
            this.mecanumPurePursuitController.deactivate();
        }
        this.isFollowing = false;
        this.mecanumPurePursuitController = null;
    }

    public void followAsync(MecanumPurePursuitController mecanumPurePursuitController) {
        this.isFollowing = true;
        this.mecanumPurePursuitController = mecanumPurePursuitController;
    }

    public void update() {
        if (this.isFollowing && !this.isFinished()) {
            this.mecanumPurePursuitController.updateOdometry();

            Position2D optimalIntersection = getOptimalIntersection(this.mecanumPurePursuitController.getCurrentPosition());

            if (optimalIntersection == null) {
                failed = true;
                return;
            }

            double differenceX = optimalIntersection.getX() - this.mecanumPurePursuitController.getCurrentPosition().getX();
            double differenceY = optimalIntersection.getY() - this.mecanumPurePursuitController.getCurrentPosition().getY();
            double differenceHeading = MathUtils.normalizeAngle(optimalIntersection.getHeadingRadians() - this.mecanumPurePursuitController.getCurrentPosition().getHeadingRadians(), 0.0);
            differenceHeading *= Constants.MECANUM_WIDTH;

            double max = Math.max(Math.abs(differenceX), Math.max(Math.abs(differenceY), Math.abs(differenceHeading)));

            if (max < 1) {
                max = 1;
            }

            double driveSpeed = this.maximumTanh * (Math.exp(this.tanhPace * max) - Math.exp(-this.tanhPace * max)) /
                    (Math.exp(-this.tanhPace * max) + Math.exp(this.tanhPace * max));

            if (Math.abs(driveSpeed) < this.minimumTanh) {
                driveSpeed = Math.signum(driveSpeed) * this.minimumTanh;
            }

            double x = differenceX / max;
            double y = differenceY / max;
            double r = differenceHeading / max;

            this.mecanumPurePursuitController.driveFieldParams(x * driveSpeed, y * driveSpeed, r * driveSpeed, mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
        } else {
            if (mecanumPurePursuitController != null) {
                this.mecanumPurePursuitController.deactivate();
            }
            this.isFollowing = false;
            this.mecanumPurePursuitController = null;
        }
    }

    public void reset() {
        if (mecanumPurePursuitController != null) {
            this.mecanumPurePursuitController.deactivate();
        }
        this.isFollowing = false;
        this.mecanumPurePursuitController = null;
    }

    public boolean isFinished() {
        return !isFollowing || this.failed() || this.nearPosition(this.waypoints.get(this.waypoints.size()-1).getEndpoint());
    }

    private boolean nearPosition(Position2D position2D) {
        return (this.mecanumPurePursuitController.getCurrentPosition().dist(position2D) <= this.positionBuffer
                && Math.abs(this.mecanumPurePursuitController.getCurrentPosition().getHeadingRadians() - position2D.getHeadingRadians()) <= rotationBuffer);
    }

    public boolean failed() {
        return failed;
    }

    public static class PurePursuitPathBuilder {
        private ArrayList<Waypoint> waypoints;

        private double followRadius;
        private double positionBuffer;
        private double rotationBuffer;

        private double tanhPace;
        private double minimumTanh;
        private double maximumTanh;

        public PurePursuitPathBuilder(StartWaypoint startWaypoint) {
            waypoints = new ArrayList<>();
            waypoints.add(startWaypoint);

            this.followRadius = 0.1;
            this.positionBuffer = 0.3;
            this.rotationBuffer = 0.3;

            this.tanhPace = 1;
            this.minimumTanh = 0;
            this.maximumTanh = 1;
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

        public PurePursuitPathBuilder setTanhPace(double tanhPace) {
            this.tanhPace = tanhPace;
            return this;
        }

        public PurePursuitPathBuilder setMinimumTanh(double minimumTanh) {
            this.minimumTanh = minimumTanh;
            return this;
        }

        public PurePursuitPathBuilder setMaximumTanh(double maximumTanh) {
            this.maximumTanh = maximumTanh;
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
