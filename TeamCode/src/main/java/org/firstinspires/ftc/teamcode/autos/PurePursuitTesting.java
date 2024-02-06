package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.StartWaypoint;

@Autonomous()
public class PurePursuitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder(new StartWaypoint(new Position2D(0, 0, 0)))
                .addWaypoint(new EndWaypoint(new Position2D(4, 0, 0)))
                .setTanhPace(0.97)
                .setMinimumTanh(0.06)
                .setMaximumTanh(0.35)
                .setFollowRadius(1)
                .setPositionBuffer(0.05)
                .setRotationBuffer(0.05)
                .build();

        waitForStart();

//        path.follow(mecanumPurePursuitController);
        path.followAsync(mecanumPurePursuitController);

        while (opModeIsActive()) {
            path.update();

            telemetry.addData("finished", path.isFinished());
            telemetry.addData("failed", path.failed());
            telemetry.addLine();
            telemetry.addData("x", mecanumPurePursuitController.getCurrentPosition().getX());
            telemetry.addData("y", mecanumPurePursuitController.getCurrentPosition().getY());
            telemetry.addData("r", mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
