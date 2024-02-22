package org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry;

public abstract class GeometricShape {
    public abstract Position2D[] circleIntersections(RestrictedCircle restrictedCircle);

    public abstract Position2D getEndpoint();
}
