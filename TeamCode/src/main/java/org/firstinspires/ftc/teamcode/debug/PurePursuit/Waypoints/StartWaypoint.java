package org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints;

public class StartWaypoint implements Waypoint {
    private Position2D position2d;

    public StartWaypoint(Position2D position2D) {
        this.position2d = position2D;
    }

    @Override
    public Position2D getPosition() {
        return this.position2d;
    }
}
