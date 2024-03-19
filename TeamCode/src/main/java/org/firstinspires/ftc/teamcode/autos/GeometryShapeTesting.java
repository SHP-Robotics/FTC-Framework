package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.GeometricShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;

//@Disabled
@Autonomous()
public class GeometryShapeTesting extends LinearOpMode {
    private void decrypt(Telemetry telemetry, Position2D position2D) {
        if (position2D == null) {
            telemetry.addLine("position is null");
            telemetry.addLine();
        } else {
            telemetry.addData("position X", position2D.getX());
            telemetry.addData("position Y", position2D.getY());
            telemetry.addData("position heading", position2D.getHeadingRadians());
            telemetry.addLine();
        }
    }

    private void decrypt(Telemetry telemetry, GeometricShape geometricShape) {
        telemetry.addLine("endpoint:");
        decrypt(telemetry, geometricShape.getEndpoint());
        if (geometricShape instanceof RestrictedCircle) {
            telemetry.addData("radius", ((RestrictedCircle) geometricShape).getRadius());
            telemetry.addLine();
        }
    }

    private void addIntersections(Telemetry telemetry, GeometricShape geometricShape, RestrictedCircle followingCircle) {
        Position2D[] intersections = geometricShape.circleIntersections(followingCircle);
        int c = 0;
        for (Position2D intersection: intersections) {
            if (intersection != null) {
                telemetry.addData("intersection " + c + " x", intersection.getX());
                telemetry.addData("intersection " + c + " y", intersection.getY());
                telemetry.addLine();
            }
            c += 1;
        }
        telemetry.addLine();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

//        while (opModeIsActive()) {
//            decrypt(telemetry, geometricShape);
//            addIntersections(telemetry, geometricShape, followingCircle);
//            telemetry.update();
//        }
    }
}
