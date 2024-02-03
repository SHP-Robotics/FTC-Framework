package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.GeneralWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;

@Autonomous()
public class PurePursuitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.4)
                .build();
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap);
        mecanumPurePursuitController.setSpeedController(speedController);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder(new StartWaypoint(new Position2D(0, 0, 0)))
//                .addWaypoint(new GeneralWaypoint(new Position2D(0, 4, 0)))
                .addWaypoint(new EndWaypoint(new Position2D(0, 4, 0)))
                .setFollowRadius(1)
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
