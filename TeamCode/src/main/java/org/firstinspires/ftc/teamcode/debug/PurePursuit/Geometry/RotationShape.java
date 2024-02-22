package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public class RotationShape extends GeometricShape {
    private Position2D center;
    private double radius;
    private double heading;

    public RotationShape(Position2D center, double radius, double heading) {
        this.center = center;
        this.radius = radius;
        this.heading = heading;
    }

    @Override
    public Position2D[] circleIntersections(RestrictedCircle restrictedCircle) {
        return new Position2D[]{
                new Position2D(
                        center.getX(),
                        center.getY(),
                        heading
                ), null
        };
    }

    @Override
    public Position2D getEndpoint() {
        return new Position2D(
                center.getX() + (radius * Math.cos(this.heading)),
                center.getY() + (radius * Math.sin(this.heading)),
                heading
        );
    }
}
