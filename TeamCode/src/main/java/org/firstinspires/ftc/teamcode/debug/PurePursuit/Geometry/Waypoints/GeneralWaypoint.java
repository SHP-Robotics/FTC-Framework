package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;

public class GeneralWaypoint implements Waypoint {
    private final Position2D endpoint;

    public GeneralWaypoint(Position2D endpoint) {
        this.endpoint = endpoint;
    }

    @Override
    public Position2D getEndpoint() {
        return this.endpoint;
    }
}
