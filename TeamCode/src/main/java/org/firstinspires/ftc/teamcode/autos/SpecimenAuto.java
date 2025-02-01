package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.generatePathFollower;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getGrabSpecimen;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getPlaceSpecimen;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SpecimenClawSubsystem;
import org.firstinspires.ftc.teamcode.SpecimenSlideSubsystem;

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends LinearOpMode {
    ThreeWheelOdometryTracker tracker;

    PathContainer startToSub;
    PathContainer subToBlocks1;
    PathContainer subToBlocks2;
    PathContainer subToBlocks3;
    PathContainer subToBlocks4;

    PathContainer blockToPick1;
    PathContainer blockToPick2;
    PathContainer blockToPick3;

    PathContainer place1;
    PathContainer place12;

    PathContainer subToSample;
    PathContainer subToSample2;
    PathContainer sampleToSub;

    PathFollower pathFollower;

    private SpecimenClawSubsystem specimenClawSubsystem;
    private SpecimenSlideSubsystem specimenSlideSubsystem;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = (ThreeWheelOdometryTracker) PestoFTCConfig.getTracker(hardwareMap);

        elapsedTime = new ElapsedTime();
        specimenClawSubsystem = new SpecimenClawSubsystem(hardwareMap);
        specimenSlideSubsystem = new SpecimenSlideSubsystem(hardwareMap);

        FourBarSubsystem fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        fourBarSubsystem.setState(FourBarSubsystem.FourBarState.UP);
        fourBarSubsystem.update();

//        ColorSensor colorSensor = (ColorSensor) hardwareMap.get("clawColor");

        Runnable placeSpecimen = getPlaceSpecimen(tracker, specimenSlideSubsystem, specimenClawSubsystem, elapsedTime, this);
        Runnable grabSpecimen = getGrabSpecimen(tracker, specimenSlideSubsystem, specimenClawSubsystem, elapsedTime, this);

        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(
                        new BezierCurve(
                            new Vector2D[]{
                                    new Vector2D(0, 0),
                                    new Vector2D(-3, -29.5)
                            }
                    )
                )
                .build();

        subToBlocks1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                    new Vector2D(-3, -29),
                                    new Vector2D(-3, -13),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-3, -13),
                                        new Vector2D(-30, -23),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-30, -23),
                                        new Vector2D(-30, -55),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-30, -55),
                                        new Vector2D(-40, -55),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-40, -55),
                                        new Vector2D(-40, 10),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .build();

        subToBlocks2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-40, 10),
                                        new Vector2D(-30, -50),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .build();

        subToBlocks3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-30, -50),
                                        new Vector2D(-53, -50),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .build();

        subToBlocks4 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-53, -50),
                                        new Vector2D(-55, 10),
                                }
                        ),
                        new ParametricHeading(new double[]{0, 0})
                )
                .build();

        blockToPick2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-55, 10),
                                        new Vector2D(-55, -30),
                                }
                        ),
                        new ParametricHeading(new double[]{0, PI + Math.toRadians(12)})
                )
                .build();

        blockToPick3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-55, -30),
                                        new Vector2D(-55, -3),
                                }
                        ),
                        new ParametricHeading(new double[]{PI + Math.toRadians(12), PI + Math.toRadians(12)})
                )
                .build();

        place1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-55, 10),
                                        new Vector2D(0, -10),
                                }
                        ),
                        new ParametricHeading(new double[]{PI + Math.toRadians(12), Math.toRadians(5)})
                )
                .build();

        place12 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, -10),
                                        new Vector2D(0, -35),
                                }
                        ),
                        new ParametricHeading(new double[]{Math.toRadians(5), Math.toRadians(5)})
                )
                .build();

        specimenClawSubsystem.setState(SpecimenClawSubsystem.ClawState.CLOSE);
        specimenClawSubsystem.update();

        waitForStart();
        tracker.reset();

        specimenSlideSubsystem.init();
        specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.HIGH);

        elapsedTime.reset();


        // --- START TO SUB ---

        pathFollower = generatePathFollower(startToSub, mecanumController, tracker)
                .addFinalAction(placeSpecimen)
                .setTimeAfterDeceleration(0.2)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(subToBlocks1, mecanumController, tracker)
                .setSpeed(0.9)
                .setEndTolerance(2, Math.toRadians(2))
                .setTimeAfterDeceleration(0.5)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(subToBlocks2, mecanumController, tracker)
                .setSpeed(0.9)
                .setEndTolerance(2, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(0.5)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(subToBlocks3, mecanumController, tracker)
                .setSpeed(0.9)
                .setEndTolerance(2, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(0.5)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(subToBlocks4, mecanumController, tracker)
                .setSpeed(0.9)
                .setEndTolerance(2, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(0.5)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        // ONE

        pathFollower = generatePathFollower(blockToPick2, mecanumController, tracker)
                .setSpeed(0.9)
                .setEndTolerance(2, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(0.5)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(blockToPick3, mecanumController, tracker)
                .setSpeed(0.3)
//                .setEndTolerance(2, Math.toRadians(2))
//                .setEndVelocityTolerance(4)
                .addFinalAction(() -> {
                    mecanumController.drive(-1.0, 0, 0);
                    ElapsedTime elapsedTime1 = new ElapsedTime();
                    elapsedTime1.reset();
                    while (elapsedTime1.milliseconds() < 2000)
                        tracker.update();
                    tracker.resetHeading(PI);
                    grabSpecimen.run();
                    elapsedTime1.reset();
                    while (elapsedTime1.milliseconds() < 1000)
                        tracker.update();
                })
                .setTimeAfterDeceleration(0.1)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(place1, mecanumController, tracker)
                .setSpeed(0.9)
                .setTimeAfterDeceleration(0.1)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(place12, mecanumController, tracker)
                .setSpeed(0.9)
                .addFinalAction(placeSpecimen)
                .setTimeAfterDeceleration(0.1)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        // TWO

        pathFollower = generatePathFollower(blockToPick2, mecanumController, tracker)
                .setSpeed(0.9)
                .setEndTolerance(2, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(0.5)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(blockToPick3, mecanumController, tracker)
                .setSpeed(0.3)
                .addFinalAction(() -> {
                    mecanumController.drive(-1.0, 0.4, 0);
                    ElapsedTime elapsedTime1 = new ElapsedTime();
                    elapsedTime1.reset();
                    while (elapsedTime1.milliseconds() < 1000)
                        tracker.update();
                    tracker.resetHeading(PI);
                    grabSpecimen.run();
                    elapsedTime1.reset();
                    while (elapsedTime1.milliseconds() < 200)
                        tracker.update();
                })
                .setKillTime(2.5)
                .setTimeAfterDeceleration(0.1)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(place1, mecanumController, tracker)
                .setSpeed(0.9)
                .setTimeAfterDeceleration(0.1)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(place12, mecanumController, tracker)
                .setSpeed(0.9)
                .addFinalAction(placeSpecimen)
                .setTimeAfterDeceleration(0.1)
                .build();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }

    public void loopOpMode() {
        tracker.update();

        pathFollower.update();

        specimenClawSubsystem.update();
        specimenSlideSubsystem.update();

        specimenClawSubsystem.updateTelemetry(telemetry);
        specimenSlideSubsystem.updateTelemetry(telemetry);

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
//        telemetry.addData("i", subToBlocks.i);
//        telemetry.addData("decel", pathFollower.isDecelerating());
//        telemetry.addData("completed", pathFollower.isCompleted());
        elapsedTime.reset();
        telemetry.update();
    }
}