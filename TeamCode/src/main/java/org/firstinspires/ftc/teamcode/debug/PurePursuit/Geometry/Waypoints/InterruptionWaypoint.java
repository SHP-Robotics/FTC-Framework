package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;

public class InterruptionWaypoint implements Waypoint {
    private final Position2D endpoint;
    private final Runnable interruptingAction;

//    private final double sleep;

    private boolean excecuted = false;

    public InterruptionWaypoint(Position2D endpoint, Runnable interruptingAction) {
        this.endpoint = endpoint;
        this.interruptingAction = interruptingAction;
//        this.sleep = 0;
    }

//    public InterruptionWaypoint(Position2D position2D, Runnable interruptingAction, int sleep) {
//        this.position2d = position2D;
//        this.interruptingAction = interruptingAction;
//        this.sleep = sleep;
//    }

    public boolean isExcecuted() {
        return this.excecuted;
    }

    public void run() {
        this.interruptingAction.run();
        this.excecuted = true;
    }

//    public double getSleep() {
//        return this.sleep;
//    }

    @Override
    public Position2D getEndpoint() {
        return this.endpoint;
    }
}
