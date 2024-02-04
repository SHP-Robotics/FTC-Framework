package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;

public class RotationWaypoint implements Waypoint {
    private final Position2D position2d;
    private final double radius;

    public RotationWaypoint(Position2D position2D, double radius) {
        this.position2d = position2D;
        this.radius = radius;
    }

    @Override
    public Position2D getPosition() {
        return this.position2d;
    }

    public double getRadius() {
        return this.radius;
    }
}
