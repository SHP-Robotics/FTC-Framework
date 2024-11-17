package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

//import org.firstinspires.ftc.teamcode.PestoFTCConfig.MecanumController;
//import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
//import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
//import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
//import org.firstinspires.ftc.teamcode.debug.config.Constants;

@Config
@Autonomous
public class TestAuto extends BaseAuto {
    public static double speed = 0.5;

    PathContainer path1, path2, path3;
    PathFollower pathFollower;
    MecanumController mecanumController;
    Tracker tracker;

    @Override
    public void init() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);

        path1 = new PathContainer.PathContainerBuilder(
                new Pose2D(0, 0, Math.toRadians(180))
        )
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(0, -30),
                                }
                        ),
                        new ParametricHeading(Math.toRadians(180), Math.toRadians(180)),
                        () -> vertical.setPosition(100)

                )
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(0,30),
//                                        new Vector2D(0, 32),
//                                }
//                        ),
//                        () -> {
//                            vertical.setPosition(50);
//                            claw.close();
//                        }
//
//                )
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(0,32),
//                                        new Vector2D(0, 30),
//                                }
//                        ),
//                        () -> vertical.setPosition(0)
//                )
                .setIncrement(0.01)
                .build();
//        path2 = new PathContainer.PathContainerBuilder(
//                new Pose2D(5, 5, 0)
//        )
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(5,5),
//                                        new Vector2D(5, 20),
//                                        new Vector2D(10, 20),
//                                        new Vector2D(10, 5),
//                                }
//                        ),
//                        new ParametricHeading(Math.toRadians(0), Math.toRadians(0))
//                )
//
//                .build();
//        path3 = new PathContainer.PathContainerBuilder(
//                new Pose2D(10, 5, 0)
//        )
//                .addCurve(
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(10,5),
//                                        new Vector2D(10, 20),
//                                        new Vector2D(10, 5),
//                                }
//                        ),
//                        new ParametricHeading(Math.toRadians(0), Math.toRadians(0))
//                )
//
//                .build();

        pathFollower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                (ThreeWheelOdometryTracker) tracker,
                path1
        )
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setEndpointPID(new PID(1, 0, 0))
                .setHeadingPID(new PID(1, 0, 0))
                .setSpeed(speed)
                .build();

        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        claw.open();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        Vector2D position = tracker.getCurrentPosition();
        double heading = tracker.getCurrentHeading();

        Pose2D velocity = tracker.getRobotVelocity();

        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("r", heading);
        telemetry.addLine();
        telemetry.addData("Δx", velocity.getX());
        telemetry.addData("Δy", velocity.getY());
        telemetry.addData("Δr", velocity.getHeadingRadians());
        telemetry.update();

        pathFollower.update();
        tracker.updateOdometry();

        super.loop();
    }
}
