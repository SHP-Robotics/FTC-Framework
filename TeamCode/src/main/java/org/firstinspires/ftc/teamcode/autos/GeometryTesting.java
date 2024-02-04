package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedLine;

@Autonomous()
public class GeometryTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RestrictedCircle restrictedCircle = new RestrictedCircle(1);
        restrictedCircle.setOffset(new Position2D(
                0.1, 0, 0
        ));

        RestrictedLine restrictedLine = new RestrictedLine(0,0, 4, 0);

        Position2D[] intersections = restrictedCircle.getLineIntersections(restrictedLine);

        if (intersections[0] == null) {
            telemetry.addLine("intersection[0] = null");
            telemetry.addLine();
        } else {
            telemetry.addData("intersections[0].getX()", intersections[0].getX());
            telemetry.addData("intersections[0].getY()", intersections[0].getY());
            telemetry.addData("intersections[0].getHeadingRadians()", intersections[0].getHeadingRadians());
            telemetry.addLine();
        }

        if (intersections[1] == null) {
            telemetry.addLine("intersection[1] = null");
            telemetry.addLine();
        } else {
            telemetry.addData("intersections[1].getX()", intersections[1].getX());
            telemetry.addData("intersections[1].getY()", intersections[1].getY());
            telemetry.addData("intersections[1].getHeadingRadians()", intersections[1].getHeadingRadians());
            telemetry.addLine();
        }

        telemetry.update();

        waitForStart();
    }
}
