package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;

public class EndWaypoint implements Waypoint {
    private final Position2D endpoint;

    public EndWaypoint(Position2D endpoint) {
        this.endpoint = endpoint;
    }

    @Override
    public Position2D getEndpoint() {
        return this.endpoint;
    }
}
