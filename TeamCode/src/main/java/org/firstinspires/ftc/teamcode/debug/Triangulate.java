package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Triangulate {
    // This implementation of triangulate uses the x axis
    // solely as the distance between the two positions.
    // TODO: DO NOT FORGET THAT...
    // ... x axis may need to be rotated to match the field...
    public static Pose2d[] Triangulate(double[] distances, double distanceBetweenPositions) throws IllegalArgumentException {
        if (distances.length != 2) {
            throw new IllegalArgumentException("Triangulate requires exactly 2 distances, but " + distances.length + " were provided.");
        }

        Pose2d[] positions = new Pose2d[2];

        double x = ((distances[0] * distances[0]) - (distances[1] * distances[1]) + (distanceBetweenPositions * distanceBetweenPositions))
                / (- 2 * distanceBetweenPositions);

        double y = Math.sqrt((distances[0] * distances[0]) - (x * x));

        positions[0] = new Pose2d(-x, y);
        positions[1] = new Pose2d(distanceBetweenPositions - x, y);

        return positions;
    }

    // This method should be used in conjunction with Triangulate to rotate the x axis to match the field
    public static Pose2d[] RotatePoints(Pose2d[] points, double angle, Pose2d origin) {
        Pose2d[] rotatedPoints = new Pose2d[points.length];

        for (int i = 0; i < points.length; i++) {
            Pose2d point = points[i];
            double x = point.getX();
            double y = point.getY();

            double rotatedX = (x * Math.cos(angle)) - (y * Math.sin(angle));
            double rotatedY = (x * Math.sin(angle)) + (y * Math.cos(angle));

            rotatedPoints[i] = new Pose2d(rotatedX, rotatedY);
        }

        return rotatedPoints;
    }
}
