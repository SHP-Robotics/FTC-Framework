package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.generatePathFollower;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getGrabSpecimen;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getPlaceSpecimen;

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

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SpecimenSubsystem;

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends LinearOpMode {
    DeterministicTracker tracker;

    PathContainer startToSub;
    PathContainer subToSample;
    PathContainer subToSample2;
    PathContainer sampleToSub;

    PathFollower pathFollower;

    private ClawSubsystem clawSubsystem;
    private SpecimenSubsystem specimenSubsystem;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);

        elapsedTime = new ElapsedTime();
        clawSubsystem = new ClawSubsystem(hardwareMap);
        specimenSubsystem = new SpecimenSubsystem(hardwareMap);

        FourBarSubsystem fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        fourBarSubsystem.setState(FourBarSubsystem.FourBarState.DOWN);
        fourBarSubsystem.update();

        Runnable placeSpecimen = getPlaceSpecimen(tracker, specimenSubsystem, clawSubsystem, elapsedTime, this);
        Runnable grabSpecimen = getGrabSpecimen(tracker, specimenSubsystem, clawSubsystem, elapsedTime, this);

        tracker = PestoFTCConfig.getTracker(hardwareMap);

        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                        new Vector2D[]{
                                new Vector2D(0, 0),
                                new Vector2D(0, -30)
                        }
                ),
                        new ParametricHeading(new double[]{0, 0}))
                .build();

        pathFollower = generatePathFollower(startToSub, mecanumController, tracker)
                .addFinalAction(placeSpecimen)
                .setTimeAfterDeceleration(0.25)
                .build();

        waitForStart();
        specimenSubsystem.init();
        specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.HIGH);

        elapsedTime.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        subToSample = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, -30),
                                        new Vector2D(-49, -20),
                                        new Vector2D(-49, 0),
                                }
                        ),
                        new ParametricHeading(new double[]{0, Math.PI, Math.PI, Math.PI, Math.PI, Math.PI, Math.PI})
                )
                .build();

        pathFollower = generatePathFollower(subToSample, mecanumController, tracker)
                .setTimeAfterDeceleration(10)
                .setEndpointPID(new PID(0.07, 0, 0.01))
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

        clawSubsystem.update();
        specimenSubsystem.update();

        clawSubsystem.updateTelemetry(telemetry);
        specimenSubsystem.updateTelemetry(telemetry);

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        elapsedTime.reset();
        telemetry.update();
    }
}