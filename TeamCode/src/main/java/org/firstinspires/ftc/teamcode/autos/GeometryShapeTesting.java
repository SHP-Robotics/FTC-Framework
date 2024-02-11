package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.GeometricShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedLine;

@Autonomous()
public class GeometryShapeTesting extends LinearOpMode {
    private void addIntersections(Telemetry telemetry, GeometricShape geometricShape, RestrictedCircle followingCircle) {
        Position2D[] intersections = geometricShape.circleIntersections(followingCircle);
        int c = 0;
        for (Position2D intersection: intersections) {
            if (intersection != null) {
                telemetry.addData("intersection " + String.valueOf(c) + " x", intersection.getX());
                telemetry.addData("intersection " + String.valueOf(c) + " y", intersection.getY());
                telemetry.addLine();
            }
            c += 1;
        }
        telemetry.addLine();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GeometricShape geometricShape = new RestrictedLine(
                new Position2D(0, 10, 0),
                new Position2D(10, 10, 0)
        );

        RestrictedCircle followingCircle = new RestrictedCircle(0.1);
        followingCircle.setOffset(new Position2D(
                0.1, 10.1, 0
        ));

        waitForStart();

        while (opModeIsActive()) {
            addIntersections(telemetry, geometricShape, followingCircle);
            telemetry.update();
        }
    }
}
