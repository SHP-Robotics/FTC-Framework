package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@Autonomous(name = "Red Auto")
public class TestAuto extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    PathContainer startToSub;
    PathContainer subToSample;
    PathContainer subToSample2;
    PathContainer sampleToSub;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;

    public PathFollower generatePathFollower(PathContainer pathContainer, Runnable finalAction, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(0.002,0, 0))
                .setHeadingPID(new PID(1.0, 0, 0))
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setSpeed(speed)

                .setEndTolerance(0.4, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(deceleration)
                .addFinalAction(finalAction)
                .build();
    }

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);

        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                        new Vector2D[]{
                                new Vector2D(0, 0),
                                new Vector2D(0, 32)
                        }
                ))
                .build();

        pathFollower = generatePathFollower(startToSub, () -> {
            elapsedTime.reset();
            while (opModeIsActive() && elapsedTime.seconds() < 0.5) {
//                slideSubsystem.update();
                tracker.update();
            }
            elapsedTime.reset();
            while (opModeIsActive() && elapsedTime.seconds() < 1.0) {
                tracker.update();
            }
            elapsedTime.reset();
            while (opModeIsActive() && elapsedTime.seconds() < 0.5) {
                tracker.update();
            }

        }, 0.25, 1.0);

        elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

//        for (int i = 0; i < 4; i++) {
//            subToSample = new PathContainer.PathContainerBuilder()
//                    .setIncrement(0.1)
//                    .addCurve(new BezierCurve(
//                                    new Vector2D[]{
//                                            new Vector2D(0, 32),
//                                            new Vector2D(49-0.3*i, 20)
//                                    }
//                            ),
//                            new ParametricHeading(0, Math.PI)
//                    )
//                    .build();
//
//            subToSample2 = new PathContainer.PathContainerBuilder()
//                    .setIncrement(0.1)
//                    .addCurve(new BezierCurve(
//                                    new Vector2D[]{
//                                            new Vector2D(49-0.3*i, 20),
//                                            new Vector2D(48.5-0.3*i, -9-0.75*i)
//                                    }
//                            ),
//                            new ParametricHeading(Math.PI, Math.PI)
//                    )
//                    .build();
//
//            sampleToSub = new PathContainer.PathContainerBuilder()
//                    .setIncrement(0.1)
//                    .addCurve(new BezierCurve(
//                                    new Vector2D[]{
//                                            new Vector2D(48.5-0.3*i, -9-0.75*i),
//                                            new Vector2D(0, 32.05)
//                                    }
//                            ),
//                            new ParametricHeading(Math.PI, 0)
//                    )
//                    .build();
//
//
//
//            pathFollower = generatePathFollower(subToSample, () -> {}, 0.75, 1.0);
//
//            while (opModeIsActive() && !pathFollower.isCompleted()) {
//                loopOpMode();
//            }
//
//            pathFollower = generatePathFollower(subToSample2, () -> {
//
//                elapsedTime.reset();
//
//                while (elapsedTime.seconds() < 0.2) {
//                    tracker.update();
//                }
//
//
//                elapsedTime.reset();
//                while (elapsedTime.seconds() < 0.5) {
//                    tracker.update();
//                }
//            }, 0.75, 0.5);
//
//            while (opModeIsActive() && !pathFollower.isCompleted()) {
//                loopOpMode();
//            }
//
//            pathFollower = generatePathFollower(sampleToSub, () -> {
//                elapsedTime.reset();
//                while (opModeIsActive() && elapsedTime.seconds() < 0.5) {
//                    tracker.update();
//                }
//                elapsedTime.reset();
//                while (opModeIsActive() && elapsedTime.seconds() < 1.0) {
//                    tracker.update();
//                }
//
//                elapsedTime.reset();
//
//                while (opModeIsActive() && elapsedTime.seconds() < 0.5) {
//                    tracker.update();
//                }
//
//            }, 0.75, 1.0);
//
//            while (opModeIsActive() && !pathFollower.isCompleted()) {
//                loopOpMode();
//            }
//        }
    }

    public void loopOpMode() {
        tracker.update();

        pathFollower.update();


        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
//        telemetry.addData("ex", subToSample.getEndpoint().getX());
//        telemetry.addData("ey", subToSample.getEndpoint().getY());
        elapsedTime.reset();
        telemetry.update();
    }
}