package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.GeometricShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.InterruptionShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedLine;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

import java.util.ArrayList;

public class PurePursuitPath {
    public final ArrayList<GeometricShape> geometries;

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
        this.geometries = purePursuitPathBuilder.geometries;

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
        int geometry = -1;

        for (int i = 0; i < this.geometries.size(); i++) {
            if (this.geometries.get(i) instanceof InterruptionShape && !((InterruptionShape) this.geometries.get(i)).isExecuted() && nearPosition(this.geometries.get(i).getEndpoint())) {
                mecanumPurePursuitController.leftFront.setPower(0);
                mecanumPurePursuitController.rightFront.setPower(0);
                mecanumPurePursuitController.leftRear.setPower(0);
                mecanumPurePursuitController.rightRear.setPower(0);
                ((InterruptionShape) this.geometries.get(i)).run();
            }

            GeometricShape geometricShape = this.geometries.get(i);

            Position2D[] tmpIntersections = geometricShape.circleIntersections(followingCircle);

            if (currentPosition.dist(geometricShape.getEndpoint()) <= followRadius) {
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
        return !isFollowing || this.failed() || this.nearPosition(this.geometries.get(this.geometries.size()-1).getEndpoint());
    }

    private boolean nearPosition(Position2D position2D) {
        return (this.mecanumPurePursuitController.getCurrentPosition().dist(position2D) <= this.positionBuffer
                && Math.abs(this.mecanumPurePursuitController.getCurrentPosition().getHeadingRadians() - position2D.getHeadingRadians()) <= rotationBuffer);
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

        private double tanhPace;
        private double minimumTanh;
        private double maximumTanh;

        public PurePursuitPathBuilder(Position2D startPosition) {
            geometries = new ArrayList<>();
            lastPosition = startPosition;

            this.followRadius = 0.1;
            this.positionBuffer = 0.3;
            this.rotationBuffer = 0.3;

            this.tanhPace = 1;
            this.minimumTanh = 0;
            this.maximumTanh = 1;
        }

        public PurePursuitPathBuilder moveTo(Position2D position2D) {
            if (position2D.getX() == this.lastPosition.getX() && position2D.getY() == this.lastPosition.getY()) {
                throw new IllegalArgumentException("position2D cannot equal the last position2D added to the path");
            }

            if (position2D.getHeadingRadians() != lastPosition.getHeadingRadians()) {
                this.geometries.add(new RestrictedCircle(lastPosition, position2D));
                if (this.geometries.get(this.geometries.size()-1).getEndpoint() != position2D) {
                    this.geometries.add(new RestrictedLine(this.geometries.get(this.geometries.size()-1).getEndpoint(), position2D));
                }
            } else {
                this.geometries.add(new RestrictedLine(lastPosition, position2D));
            }

            lastPosition = position2D;

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
