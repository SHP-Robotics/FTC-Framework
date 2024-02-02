package org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints;

public class EndWaypoint implements Waypoint {
    private Position2D position2d;

    public EndWaypoint(Position2D position2D) {
        this.position2d = position2D;
    }

    @Override
    public Position2D getPosition() {
        return this.position2d;
    }
}
