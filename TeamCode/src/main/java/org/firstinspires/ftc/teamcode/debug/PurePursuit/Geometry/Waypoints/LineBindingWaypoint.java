package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;

public class LineBindingWaypoint implements Waypoint {
    private Position2D endpoint;

    public LineBindingWaypoint(Position2D endpoint) {
        this.endpoint = endpoint;
    }

    @Override
    public Position2D getEndpoint() {
        return this.endpoint;
    }
}
