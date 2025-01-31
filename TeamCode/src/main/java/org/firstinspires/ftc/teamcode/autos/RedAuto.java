package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.BELOW_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;

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
import org.firstinspires.ftc.teamcode.SlideSubsystem;
import org.firstinspires.ftc.teamcode.SpecimenClawSubsystem;

@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    PathContainer startToSub;
    PathContainer subToSample;
    PathContainer subToSample2;
    PathContainer sampleToSub;
    PathContainer sampleToSub2;

    PathFollower pathFollower;

    private SpecimenClawSubsystem clawSubsystem;
    private SlideSubsystem slideSubsystem;

    private ElapsedTime elapsedTime;

    public PathFollower generatePathFollower(PathContainer pathContainer, Runnable finalAction, double decelerationSpeed, double decelerationTime, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(0.012,0, 0))//0.008
                .setHeadingPID(new PID(0.8, 0, 0))
                .setDeceleration(decelerationSpeed)
                .setSpeed(speed)

                .setEndTolerance(0.4, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(decelerationTime)
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
                                new Vector2D(0, -22)
                        }
                ))
                .build();

        pathFollower = generatePathFollower(startToSub, () -> {
            elapsedTime.reset();
            while (opModeIsActive() && elapsedTime.seconds() < 0.5)
                slideSubsystem.update();
            slideSubsystem.setState(BELOW_HIGH_RUNG);
            elapsedTime.reset();
            while (opModeIsActive() && elapsedTime.seconds() < 1.0)
                slideSubsystem.update();

            elapsedTime.reset();
            clawSubsystem.setState(SpecimenClawSubsystem.ClawState.OPEN);

            while (opModeIsActive() && elapsedTime.seconds() < 0.5)
                clawSubsystem.update();

            slideSubsystem.setState(INTAKE);
        }, PestoFTCConfig.DECELERATION, 0.25, 0.6);

        clawSubsystem = new SpecimenClawSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        elapsedTime = new ElapsedTime();

        waitForStart();
        slideSubsystem.init();
        slideSubsystem.setState(ABOVE_HIGH_RUNG);

        elapsedTime.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

       // for (int i = 0; i < 3; i++) {
            subToSample = new PathContainer.PathContainerBuilder()
                    .setIncrement(0.1)
                    .addCurve(new BezierCurve(
                                    new Vector2D[]{
                                            new Vector2D(0, -32),
                                            new Vector2D(-49, -20)
                                    }
                            ),
                            new ParametricHeading(0, Math.PI)
                    )
                    .build();

            subToSample2 = new PathContainer.PathContainerBuilder()
                    .setIncrement(0.1)
                    .addCurve(new BezierCurve(
                                    new Vector2D[]{
                                            new Vector2D(-49, -20),
                                            new Vector2D(-49, 13.5) //12.5
                                    }
                            ),
                            new ParametricHeading(Math.PI, Math.PI)
                    )
                    .build();

            sampleToSub = new PathContainer.PathContainerBuilder()
                    .setIncrement(0.1)
                    .addCurve(new BezierCurve(
                                    new Vector2D[]{
                                            new Vector2D(-49, 12.5),
                                            new Vector2D(0, 10)
                                    }
                            ),
                            new ParametricHeading(Math.PI, 0)
                    )
                    .build();

            sampleToSub2 = new PathContainer.PathContainerBuilder()
                    .setIncrement(0.1)
                    .addCurve(new BezierCurve(
                                    new Vector2D[]{
                                            new Vector2D(0, 10),
                                            new Vector2D(0, -34.5)
                                    }
                            ),
                            new ParametricHeading(0, 0)
                    )
                    .build();

            pathFollower = generatePathFollower(subToSample, () -> {}, PestoFTCConfig.DECELERATION,1.5, 0.6);

            while (opModeIsActive() && !pathFollower.isCompleted()) {
                loopOpMode();
            }

            mecanumController.drive(0, 0, 0);
            sleep(500);

            pathFollower = generatePathFollower(subToSample2, () -> {
                clawSubsystem.setState(SpecimenClawSubsystem.ClawState.CLOSE);
                clawSubsystem.update();

                elapsedTime.reset();

                while (elapsedTime.seconds() < 0.2) {
                }

                slideSubsystem.setState(ABOVE_HIGH_RUNG);

                elapsedTime.reset();
                while (elapsedTime.seconds() < 0.5)
                    slideSubsystem.update();
            },PestoFTCConfig.DECELERATION, Double.POSITIVE_INFINITY, 0.6);

            ElapsedTime elapsedTime1 = new ElapsedTime();
            elapsedTime1.reset();
            while (opModeIsActive() && !pathFollower.isCompleted() && elapsedTime1.seconds() < 7) {
                loopOpMode();
            }
            mecanumController.drive(0, 0, 0);

           // pathFollower = generatePathFollower(sampleToSub, () -> {}, PestoFTCConfig.DECELERATION,0.8, 0.6);

//            while (opModeIsActive() && !pathFollower.isCompleted()) {
//                loopOpMode();
//            }

//            mecanumController.drive(0, 0, 0);
//            sleep(500);
//
//            pathFollower = generatePathFollower(sampleToSub2, () -> {
//                elapsedTime.reset();
//                while (opModeIsActive() && elapsedTime.seconds() < 0.5)
//                    slideSubsystem.update();
//                slideSubsystem.setState(BELOW_HIGH_RUNG);
//                elapsedTime.reset();
//                while (opModeIsActive() && elapsedTime.seconds() < 1.0)
//                    slideSubsystem.update();
//
//                elapsedTime.reset();
//                clawSubsystem.setState(SpecimenClawSubsystem.ClawState.OPEN);
//
//                while (opModeIsActive() && elapsedTime.seconds() < 0.5)
//                    clawSubsystem.update();
//
//                slideSubsystem.setState(INTAKE);
//            }, PestoFTCConfig.DECELERATION,2.0, 0.6);
//
//            while (opModeIsActive() && !pathFollower.isCompleted()) {
//                loopOpMode();
//            }
       // }
    }

    public void loopOpMode() {
        tracker.update();

        pathFollower.update();

        clawSubsystem.update();
        slideSubsystem.update();

        clawSubsystem. updateTelemetry(telemetry);
        slideSubsystem.updateTelemetry(telemetry);

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
