package org.firstinspires.ftc.teamcode.PurePursuit.Geometry;

public abstract class GeometricShape {
    public abstract Position2D[] circleIntersections(RestrictedCircle restrictedCircle);

    public abstract Position2D getEndpoint();
}
