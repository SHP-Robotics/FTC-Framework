package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public class InterruptionShape extends GeometricShape {
    private final Position2D endpoint;
    private final Runnable runnable;
    private boolean isExecuted;

    public InterruptionShape(Position2D position2D, Runnable runnable) {
        this.endpoint = position2D;
        this.runnable = runnable;
        this.isExecuted = false;
    }

    @Override
    public Position2D[] circleIntersections(RestrictedCircle restrictedCircle) {
        return null;
    }

    @Override
    public Position2D getEndpoint() {
        return this.endpoint;
    }

    public void run() {
        runnable.run();
        this.isExecuted = true;
    }

    public boolean isExecuted() {
        return this.isExecuted;
    }
}
