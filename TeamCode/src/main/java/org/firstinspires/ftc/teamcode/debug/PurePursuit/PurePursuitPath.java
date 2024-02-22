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
    public final ArrayList<Double> admissibleXYErrors;
    public final ArrayList<Double> admissibleRotationalErrors;

    private int currentGeometry = 0;

    private final double followRadius;
    private final double positionBuffer;
    private final double rotationBuffer;

    private final double tanhPace;
    private final double minimumTanh;
    private final double maximumTanh;

    private boolean isFollowing = false;
    private boolean failed = false;
    private PurePursuitFollower purePursuitFollower;
    private RobotController robotController;

    public PurePursuitPath(PurePursuitPathBuilder purePursuitPathBuilder) {
        this.geometries = purePursuitPathBuilder.geometries;
        this.admissibleXYErrors = purePursuitPathBuilder.admissibleXYErrors;
        this.admissibleRotationalErrors = purePursuitPathBuilder.admissibleRotationalErrors;

        this.followRadius = purePursuitPathBuilder.followRadius;
        this.positionBuffer = purePursuitPathBuilder.positionBuffer;
        this.rotationBuffer = purePursuitPathBuilder.rotationBuffer;

        this.tanhPace = purePursuitPathBuilder.tanhPace;
        this.minimumTanh = purePursuitPathBuilder.minimumTanh;
        this.maximumTanh = purePursuitPathBuilder.maximumTanh;
    }

    public int getCurrentGeometry() {
        return this.currentGeometry;
    }

    private Position2D getGeometryIntersection(Position2D currentPosition, RestrictedCircle followingCircle, GeometricShape geometricShape) {
        Position2D[] tmpIntersections = geometricShape.circleIntersections(followingCircle);

        if (geometricShape.getEndpoint() != null && currentPosition.dist(geometricShape.getEndpoint()) <= followRadius) {
            return geometricShape.getEndpoint();
        } else if (tmpIntersections != null && tmpIntersections.length == 2 && (tmpIntersections[0] != null || tmpIntersections[1] != null)) {
            if (tmpIntersections[0] == null) {
                return tmpIntersections[1];
            }

            if (tmpIntersections[1] == null) {
                return tmpIntersections[0];
            }

            if (geometricShape.getEndpoint().dist(tmpIntersections[0]) < geometricShape.getEndpoint().dist(tmpIntersections[1])) {
                return tmpIntersections[0];
            }

            return tmpIntersections[1];
        }

        return null;
    }

    public Position2D getOptimalIntersection(Position2D currentPosition) {
        RestrictedCircle followingCircle = new RestrictedCircle(followRadius);
        followingCircle.setOffset(currentPosition);

        GeometricShape geometricShape;
        double admissibleXYError;
        double admissibleRotationalError;

        while (true) {
            if (currentGeometry >= geometries.size()) {
                return null;
            }

            geometricShape = geometries.get(currentGeometry);
            admissibleXYError = admissibleXYErrors.get(currentGeometry);
            admissibleRotationalError = admissibleRotationalErrors.get(currentGeometry);

            if (geometricShape instanceof InterruptionShape) {
                if (nearPosition(geometricShape.getEndpoint(), admissibleXYError, admissibleRotationalError) && !((InterruptionShape) geometries.get(currentGeometry)).isExecuted()) {
                    ((InterruptionShape) (geometries.get(currentGeometry))).run();
                    currentGeometry += 1;
                } else {
                    break;
                }
            } else if (nearPosition(geometricShape.getEndpoint(), admissibleXYError, admissibleRotationalError)) {
                currentGeometry += 1;
            } else {
                break;
            }
        }

        return getGeometryIntersection(currentPosition, followingCircle, geometricShape);
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
            Position2D currentPosition = this.purePursuitFollower.getCurrentPosition();

            Position2D optimalIntersection = getOptimalIntersection(currentPosition);

            if (optimalIntersection == null) {
                this.robotController.deactivate();
                failed = true;
                return;
            }

            double differenceX = optimalIntersection.getX() - currentPosition.getX();
            double differenceY = optimalIntersection.getY() - currentPosition.getY();
            double differenceHeading = -MathUtils.normalizeAngle(optimalIntersection.getHeadingRadians() - currentPosition.getHeadingRadians(), 0);
            differenceHeading *= Constants.MECANUM_WIDTH / 2;

            Position2D a = new Position2D(differenceX, differenceY, differenceHeading);
            Position2D b = purePursuitFollower.getRobotDeltaPosition();

            // TODO: pat Theo on the back
            // BIG BRAIN?!
            if (Math.abs(Math.cosh( ((a.getX()*b.getX()) + (a.getY()*b.getY())) / (a.getMagnitude() * b.getMagnitude()) )) > Math.toRadians(1)) {
                Position2D optimalVelocity = VelocityApproximator.getScaledTargetVelocity(a, b);
                differenceX = optimalVelocity.getX();
                differenceY = optimalVelocity.getY();
                differenceHeading = optimalVelocity.getHeadingRadians();
            }

            double max = Math.abs(differenceX) + Math.abs(differenceY) + Math.abs(differenceHeading);

            if (max < 1) {
                max = 1;
            }

            double driveSpeed;

            if (this.maximumTanh == this.minimumTanh) {
                driveSpeed = this.minimumTanh;
            } else {
                driveSpeed = this.maximumTanh * (Math.exp(this.tanhPace * max) - Math.exp(-this.tanhPace * max)) /
                        (Math.exp(-this.tanhPace * max) + Math.exp(this.tanhPace * max));

                if (Math.abs(driveSpeed) < this.minimumTanh) {
                    driveSpeed = Math.signum(driveSpeed) * this.minimumTanh;
                }
            }

            double x = differenceX / max;
            double y = differenceY / max;
            double r = differenceHeading / max;

            if (this.robotController instanceof SimulatedMecanumController) {
                ((SimulatedMecanumController)this.robotController).simulateEncoders(this.purePursuitFollower.getRobotDeltaPosition());
            }

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
        return !isFollowing || this.failed() || (this.currentGeometry == this.geometries.size()-1 && this.nearPosition(this.geometries.get(this.geometries.size()-1).getEndpoint()));
    }

    public boolean nearPosition(Position2D position2D) {
        return (this.purePursuitFollower.getCurrentPosition().dist(position2D) <= this.positionBuffer
                && Math.abs(this.purePursuitFollower.getCurrentPosition().getHeadingRadians() - position2D.getHeadingRadians()) <= rotationBuffer);
    }

    public boolean nearPosition(Position2D position2D, double admissibleXYError, double admissibleRotationalError) {
        return (this.purePursuitFollower.getCurrentPosition().dist(position2D) <= admissibleXYError
                && Math.abs(this.purePursuitFollower.getCurrentPosition().getHeadingRadians() - position2D.getHeadingRadians()) <= admissibleRotationalError);
    }

    public boolean failed() {
        return failed;
    }

    public static class PurePursuitPathBuilder {
        private final ArrayList<GeometricShape> geometries;
        private final ArrayList<Double> admissibleXYErrors;
        private final ArrayList<Double> admissibleRotationalErrors;
        private Position2D lastPosition;

        private double followRadius;
        private double positionBuffer;
        private double rotationBuffer;

        private double tanhPace;
        private double minimumTanh;
        private double maximumTanh;

        public PurePursuitPathBuilder() {
            geometries = new ArrayList<>();
            admissibleXYErrors = new ArrayList<>();
            admissibleRotationalErrors = new ArrayList<>();
            lastPosition = new Position2D(0, 0, Math.toRadians(90));

            this.followRadius = Constants.followRadius;
            this.positionBuffer = Constants.positionBuffer;
            this.rotationBuffer = Constants.rotationBuffer;

            this.tanhPace = Constants.tanhPace;
            this.minimumTanh = Constants.minimumTanh;
            this.maximumTanh = Constants.maximumTanh;
        }

        public PurePursuitPathBuilder moveTo(Position2D position2D, double admissibleXYError, double admissibleRotationalError) {
            if (position2D.getX() == this.lastPosition.getX() && position2D.getY() == this.lastPosition.getY()) {
                throw new IllegalArgumentException("position2D cannot equal the last position2D added to the path");
            }

            if (position2D.getHeadingRadians() != lastPosition.getHeadingRadians()) {
                throw new IllegalArgumentException("position2D heading must equal the last position2D heading");
            }

            this.geometries.add(new RestrictedLine(lastPosition, position2D));
            this.admissibleXYErrors.add(admissibleXYError);
            this.admissibleRotationalErrors.add(admissibleRotationalError);
            lastPosition = position2D;

            return this;
        }

        public PurePursuitPathBuilder moveTo(Position2D position2D) {
            return this.moveTo(position2D, Constants.followRadius, Constants.rotationBuffer);
        }
        
        public PurePursuitPathBuilder rotateTo(Position2D pointOfRotation, double heading, double admissibleXYError, double admissibleRotationalError) {
            if (heading == lastPosition.getHeadingRadians()) {
                throw new IllegalArgumentException("heading cannot equal last heading");
            }
            this.geometries.add(new RotationShape(pointOfRotation, 0, heading));
            this.admissibleXYErrors.add(admissibleXYError);
            this.admissibleRotationalErrors.add(admissibleRotationalError);
            lastPosition = this.geometries.get(this.geometries.size()-1).getEndpoint();
            return this;
        }

        public PurePursuitPathBuilder rotateTo(Position2D pointOfRotation, double heading) {
            return this.rotateTo(pointOfRotation, heading, Constants.followRadius, Constants.rotationBuffer);
        }

        public PurePursuitPathBuilder turnTo(Position2D position2D, double admissibleXYError, double admissibleRotationalError) {
            this.geometries.add(new RestrictedCircle(lastPosition, position2D));
            this.admissibleXYErrors.add(admissibleXYError);
            this.admissibleRotationalErrors.add(admissibleRotationalError);
            if (this.geometries.get(this.geometries.size()-1).getEndpoint() != position2D) {
                this.geometries.add(new RestrictedLine(this.geometries.get(this.geometries.size()-1).getEndpoint(), position2D));
                this.admissibleXYErrors.add(admissibleXYError);
                this.admissibleRotationalErrors.add(admissibleRotationalError);
            }
            return this;
        }

        public PurePursuitPathBuilder turnTo(Position2D position2D) {
            return this.turnTo(position2D, Constants.followRadius, Constants.rotationBuffer);
        }

        public PurePursuitPathBuilder addAction(double admissibleXYError, double admissibleRotationalError, Runnable runnable) {
            this.geometries.add(new InterruptionShape(lastPosition, runnable));
            this.admissibleXYErrors.add(admissibleXYError);
            this.admissibleRotationalErrors.add(admissibleRotationalError);
            return this;
        }

        public PurePursuitPathBuilder addAction(Runnable runnable) {
            return this.addAction(Constants.followRadius, Constants.rotationBuffer, runnable);
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

        public PurePursuitPathBuilder disableTanh(double driveSpeed) {
            this.tanhPace = 0;
            this.minimumTanh = driveSpeed;
            this.maximumTanh = driveSpeed;
            return this;
        }

        public PurePursuitPathBuilder enableTanh(double tanhPace, double minimumTanh, double maximumTanh) {
            this.tanhPace = tanhPace;
            this.minimumTanh = minimumTanh;
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
