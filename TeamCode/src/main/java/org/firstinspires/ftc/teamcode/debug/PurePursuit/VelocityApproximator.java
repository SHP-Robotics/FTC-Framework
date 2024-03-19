package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

public class VelocityApproximator {
//    private static Position2D convertToMecanumDriveTrain(Position2D deltaPosition) {
//        return new Position2D(
//                deltaPosition.getX() - (deltaPosition.getHeadingRadians() * Constants.MECANUM_OFFSET),
//                deltaPosition.getY(),
//                deltaPosition.getHeadingRadians()
//        );
//    }

    public static double[] getVelocities(Position2D deltaPosition, double deltaTime) {
        double x = deltaPosition.getX();
        double y = deltaPosition.getY();
        double r = deltaPosition.getHeadingRadians();
        r *= Constants.MECANUM_WIDTH / 2;

        return new double[]{
                (y + x + r) / deltaTime,
                (y - x - r) / deltaTime,
                (y - x + r) / deltaTime,
                (y + x - r) / deltaTime
        };
    }

    public static Position2D getScaledTargetVelocity(Position2D targetVelocity, Position2D currentVelocity) {
//        return Position2D.add(targetVelocity, currentVelocity.getNegative());
        return Position2D.add(targetVelocity, Position2D.multiply(currentVelocity, 1/Constants.MAX_VELOCITY).getNegative());
    }

    public static double[] getBottlenecks(double[] targetVelocities, double[] currentVelocities) {
        double[] velocityErrors = new double[]{
                VelocityApproximator.velocityError(targetVelocities[0], currentVelocities[0]),
                VelocityApproximator.velocityError(targetVelocities[1], currentVelocities[1]),
                VelocityApproximator.velocityError(targetVelocities[2], currentVelocities[2]),
                VelocityApproximator.velocityError(targetVelocities[3], currentVelocities[3])
        };

        double max = VelocityApproximator.getMaxVelocity(velocityErrors);

        for (int i = 0; i < 4; i++) {
            velocityErrors[i] /= (max * max / Constants.MAX_VELOCITY);
        }

        return velocityErrors;
    }

    public static double velocityError(double targetVelocity, double currentVelocity) {
        return Math.abs(targetVelocity - currentVelocity);
    }

    public static double getMaxVelocity(double[] velocities) {
        double max = 0;
        for (double velocity: velocities) {
            if (Math.abs(velocity) > max) {
                max = Math.abs(velocity);
            }
        }
        return max;
    }
}
