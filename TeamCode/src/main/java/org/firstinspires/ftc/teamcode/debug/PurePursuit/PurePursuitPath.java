package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.GeometricShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.InterruptionShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedLine;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RotationShape;
import org.firstinspires.ftc.teamcode.debug.RobotController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

import java.util.ArrayList;

public class PurePursuitPath {
    public final ArrayList<GeometricShape> geometries;

    private final double followRadius;
    private final double positionBuffer;
    private final double rotationBuffer;

    private final double speedMin;
    private final double speedCap;
    private final double pace;

    private boolean isFollowing = false;
    private boolean failed = false;
    private PurePursuitFollower purePursuitFollower;
    private RobotController robotController;

    public PurePursuitPath(PurePursuitPathBuilder purePursuitPathBuilder) {
        this.geometries = purePursuitPathBuilder.geometries;

        this.followRadius = purePursuitPathBuilder.followRadius;
        this.positionBuffer = purePursuitPathBuilder.positionBuffer;
        this.rotationBuffer = purePursuitPathBuilder.rotationBuffer;

        this.speedMin = purePursuitPathBuilder.speedMin;
        this.speedCap = purePursuitPathBuilder.speedCap;
        this.pace = purePursuitPathBuilder.pace;
    }

    private Position2D getOptimalIntersection(Position2D currentPosition) {
        RestrictedCircle followingCircle = new RestrictedCircle(followRadius);
        followingCircle.setOffset(currentPosition);

        Position2D[] furthestIntersections = new Position2D[2];
        int geometry = -1;

        for (int i = 0; i < this.geometries.size(); i++) {
            if (this.geometries.get(i) instanceof InterruptionShape && !((InterruptionShape) this.geometries.get(i)).isExecuted() && nearPosition(this.geometries.get(i).getEndpoint())) {
                robotController.deactivate();
                ((InterruptionShape) this.geometries.get(i)).run();
            }

            GeometricShape geometricShape = this.geometries.get(i);

            Position2D[] tmpIntersections = geometricShape.circleIntersections(followingCircle);

            if (geometricShape.getEndpoint() != null && currentPosition.dist(geometricShape.getEndpoint()) <= followRadius) {
                furthestIntersections = new Position2D[]{geometricShape.getEndpoint(), null};
                geometry = i;
            } else if (tmpIntersections != null && tmpIntersections.length == 2 && (tmpIntersections[0] != null || tmpIntersections[1] != null)) {
                furthestIntersections = tmpIntersections;
                geometry = i;
            }
        }

        if (furthestIntersections[0] == null) {
            return furthestIntersections[1];
        }

        if (furthestIntersections[1] == null) {
            return furthestIntersections[0];
        }

        if (geometries.get(geometry).getEndpoint().dist(furthestIntersections[0]) < geometries.get(geometry).getEndpoint().dist(furthestIntersections[1])) {
            return furthestIntersections[0];
        }

        return furthestIntersections[1];
    }

    public void follow(PurePursuitFollower purePursuitFollower, RobotController robotController) {
        this.isFollowing = true;
        this.purePursuitFollower = purePursuitFollower;
        this.robotController = robotController;
        while (!this.isFinished()) {
            update();
        }
        if (robotController != null) {
            this.robotController.deactivate();
        }
        this.isFollowing = false;
        this.purePursuitFollower = null;
        this.robotController = null;
    }

    public void followAsync(PurePursuitFollower purePursuitFollower, RobotController robotController) {
        this.isFollowing = true;
        this.purePursuitFollower = purePursuitFollower;
        this.robotController = robotController;
    }

    public void update() {
        if (this.isFollowing && !this.isFinished()) {
            this.purePursuitFollower.updateOdometry();

            Position2D optimalIntersection = getOptimalIntersection(this.purePursuitFollower.getCurrentPosition());

            if (optimalIntersection == null) {
                failed = true;
                return;
            }

            double differenceX = optimalIntersection.getX() - this.purePursuitFollower.getCurrentPosition().getX();
            double differenceY = optimalIntersection.getY() - this.purePursuitFollower.getCurrentPosition().getY();
            double differenceHeading = MathUtils.normalizeAngle(optimalIntersection.getHeadingRadians() - this.purePursuitFollower.getCurrentPosition().getHeadingRadians(), Math.PI/2);
            differenceHeading *= Constants.MECANUM_WIDTH;

            double max = Math.max(Math.abs(differenceX), Math.max(Math.abs(differenceY), Math.abs(differenceHeading)));

            double driveSpeed = Math.max(this.speedMin, Math.min(max * this.pace, this.speedCap));

            if (max < 1) {
                max = 1;
            }

            double x = differenceX / max;
            double y = differenceY / max;
            double r = differenceHeading / max;

            this.robotController.driveFieldParams(x * driveSpeed, y * driveSpeed, r * driveSpeed, purePursuitFollower.getCurrentPosition().getHeadingRadians());
        } else {
            if (robotController != null) {
                this.robotController.deactivate();
            }
            this.isFollowing = false;
            this.purePursuitFollower = null;
        }
    }

    public void reset() {
        if (robotController != null) {
            this.robotController.deactivate();
        }
        this.isFollowing = false;
        this.purePursuitFollower = null;
    }

    public boolean isFinished() {
        return !isFollowing || this.failed() || this.nearPosition(this.geometries.get(this.geometries.size()-1).getEndpoint());
    }

    private boolean nearPosition(Position2D position2D) {
        return (this.purePursuitFollower.getCurrentPosition().dist(position2D) <= this.positionBuffer
                && Math.abs(this.purePursuitFollower.getCurrentPosition().getHeadingRadians() - position2D.getHeadingRadians()) <= rotationBuffer);
    }

    public boolean failed() {
        return failed;
    }

    public static class PurePursuitPathBuilder {
        private ArrayList<GeometricShape> geometries;
        private Position2D lastPosition;

        private double followRadius;
        private double positionBuffer;
        private double rotationBuffer;

        private double speedMin;
        private double speedCap;
        private double pace;

        public PurePursuitPathBuilder() {
            geometries = new ArrayList<>();
            lastPosition = new Position2D(0, 0, Math.toRadians(90));

            this.followRadius = 0.1;
            this.positionBuffer = 0.3;
            this.rotationBuffer = 0.3;

            this.speedMin = 0;
            this.speedCap = 1;
            this.pace = 0.02;
        }

        public PurePursuitPathBuilder moveTo(Position2D position2D) {
            if (position2D.getX() == this.lastPosition.getX() && position2D.getY() == this.lastPosition.getY()) {
                throw new IllegalArgumentException("position2D cannot equal the last position2D added to the path");
            }

            if (position2D.getHeadingRadians() != lastPosition.getHeadingRadians()) {
                throw new IllegalArgumentException("position2D heading must equal the last position2D heading");
            } else {
                this.geometries.add(new RestrictedLine(lastPosition, position2D));
            }

            lastPosition = position2D;

            return this;
        }
        
        public PurePursuitPathBuilder rotateTo(Position2D pointOfRotation, double heading) {
            if (heading == lastPosition.getHeadingRadians()) {
                throw new IllegalArgumentException("heading cannot equal last heading");
            }
            this.geometries.add(new RotationShape(pointOfRotation, 0, heading));
            return this;
        }

        public PurePursuitPathBuilder turnTo(Position2D position2D) {
            this.geometries.add(new RestrictedCircle(lastPosition, position2D));
            if (this.geometries.get(this.geometries.size()-1).getEndpoint() != position2D) {
                this.geometries.add(new RestrictedLine(this.geometries.get(this.geometries.size()-1).getEndpoint(), position2D));
            }
            return this;
        }

        public PurePursuitPathBuilder addAction(Runnable runnable) {
            this.geometries.add(new InterruptionShape(lastPosition, runnable));
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

        public PurePursuitPathBuilder setSpeedMin(double speedMin) {
            this.speedMin = speedMin;
            return this;
        }

        public PurePursuitPathBuilder setSpeedCap(double speedCap) {
            this.speedCap = speedCap;
            return this;
        }

        public PurePursuitPathBuilder setPace(double pace) {
            this.pace = pace;
            return this;
        }

        private void compile() {
            if (geometries.size() < 1) {
                throw new IllegalArgumentException("Pure Pursuit Paths must include two points");
            }
        }

        public PurePursuitPath build() {
            compile();

            return new PurePursuitPath(this);
        }
    }
}
