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
                -(y + x + r) / deltaTime,
                -(y - x - r) / deltaTime,
                -(y - x + r) / deltaTime,
                -(y + x - r) / deltaTime
        };
    }

    public static Position2D getScaledTargetVelocity(Position2D targetVelocity, Position2D currentVelocity) {
        return Position2D.add(targetVelocity, currentVelocity.getNegative());
    }

    public static double getBottleneck(double[] velocities) {
        double min = Math.abs(velocities[0]);
        for (double velocity: velocities) {
            if (Math.abs(velocity) < min) {
                min = Math.abs(velocity);
            }
        }
        return min;
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
