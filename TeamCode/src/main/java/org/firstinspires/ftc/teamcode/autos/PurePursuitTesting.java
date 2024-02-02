package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Waypoints.StartWaypoint;

@Autonomous()
public class PurePursuitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final double MECANUM_WIDTH = 14;
        final double ODOMETRY_WIDTH = 11;
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap, MECANUM_WIDTH, ODOMETRY_WIDTH);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder(new StartWaypoint(new Position2D(0, 0, 0)))
                .addWaypoint(new EndWaypoint(new Position2D(0, 0, 0)))
                .build();

        waitForStart();

//        path.follow(mecanumPurePursuitController);
        path.followAsync(mecanumPurePursuitController);

        while (opModeIsActive()) {
            path.update();

            telemetry.addData("x", mecanumPurePursuitController.getCurrentPosition().getX());
            telemetry.addData("y", mecanumPurePursuitController.getCurrentPosition().getY());
            telemetry.addData("r", mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
