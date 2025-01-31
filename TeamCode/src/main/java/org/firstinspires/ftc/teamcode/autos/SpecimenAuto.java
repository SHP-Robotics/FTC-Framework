package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.generatePathFollower;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getGrabSpecimen;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getPlaceSpecimen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
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
    DeterministicTracker tracker;

    PathContainer startToSub;
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
        tracker = PestoFTCConfig.getTracker(hardwareMap);

        elapsedTime = new ElapsedTime();
        specimenClawSubsystem = new SpecimenClawSubsystem(hardwareMap);
        specimenSlideSubsystem = new SpecimenSlideSubsystem(hardwareMap);

        FourBarSubsystem fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        fourBarSubsystem.setState(FourBarSubsystem.FourBarState.UP);
        fourBarSubsystem.update();

        Runnable placeSpecimen = getPlaceSpecimen(tracker, specimenSlideSubsystem, specimenClawSubsystem, elapsedTime, this);
        Runnable grabSpecimen = getGrabSpecimen(tracker, specimenSlideSubsystem, specimenClawSubsystem, elapsedTime, this);

        tracker = PestoFTCConfig.getTracker(hardwareMap);

        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                        new Vector2D[]{
                                new Vector2D(0, 0),
                                new Vector2D(0, -29)
                        }
                ),
                        new ParametricHeading(new double[]{0, 0}))
                .build();

        pathFollower = generatePathFollower(startToSub, mecanumController, tracker)
                .addFinalAction(placeSpecimen)
                .setTimeAfterDeceleration(0.25)
                .build();

        waitForStart();
        specimenSlideSubsystem.init();
        specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.HIGH);

        elapsedTime.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        subToSample = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, -29),
                                        new Vector2D(-49, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{0, Math.PI, Math.PI, Math.PI, Math.PI})
                )
                .build();

        subToSample2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-49, -20),
                                        new Vector2D(-49, 0)
                                }
                        ),
                        new ParametricHeading(new double[]{Math.PI, Math.PI, Math.PI, Math.PI, Math.PI, Math.PI})
                )
                .build();

        pathFollower = generatePathFollower(subToSample, mecanumController, tracker)
                .setTimeAfterDeceleration(3)
                .build();
        pathFollower.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        pathFollower = generatePathFollower(subToSample2, mecanumController, tracker)
                .setTimeAfterDeceleration(3)
                .build();
        pathFollower.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

//        pathFollower = generatePathFollower(subToSample2, mecanumController, tracker)
//                .addFinalAction(grabSpecimen)
//                .setEndTolerance(0.4, Math.toRadians(2))
//                .setSpeed(0.5)
//                .build();
//        pathFollower.reset();
//
//        while (opModeIsActive() && !pathFollower.isCompleted()) {
//            loopOpMode();
//        }
//
//        pathFollower = generatePathFollower(sampleToSub, mecanumController, tracker)
//                .addFinalAction(placeSpecimen)
//                .setTimeAfterDeceleration(0.75)
//                .build();
//
//        while (opModeIsActive() && !pathFollower.isCompleted()) {
//            loopOpMode();
//        }
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
        telemetry.addData("r", subToSample2);
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        elapsedTime.reset();
        telemetry.update();
    }
}