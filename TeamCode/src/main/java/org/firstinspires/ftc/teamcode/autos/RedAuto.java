package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.generatePathFollower;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getGrabSpecimen;
import static org.firstinspires.ftc.teamcode.PestoFTCConfig.getPlaceSpecimen;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.ABOVE_HIGH_RUNG;

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

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SlideSubsystem;

@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode {
    private DeterministicTracker tracker;
    PathContainer startToSub;
    PathContainer subToSample;
    PathContainer subToSample2;
    PathContainer sampleToSub;

    PathFollower pathFollower;

    private ClawSubsystem clawSubsystem;
    private SlideSubsystem slideSubsystem;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Runnable placeSpecimen = getPlaceSpecimen(tracker, slideSubsystem, clawSubsystem, elapsedTime, this);
        Runnable grabSpecimen = getGrabSpecimen(tracker, slideSubsystem, clawSubsystem, elapsedTime, this);

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

        pathFollower = generatePathFollower(startToSub, mecanumController, tracker)
                .addFinalAction(placeSpecimen)
                .setTimeAfterDeceleration(0.25)
                .build();

        clawSubsystem = new ClawSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        elapsedTime = new ElapsedTime();

        waitForStart();
        slideSubsystem.init();
        slideSubsystem.setState(ABOVE_HIGH_RUNG);

        elapsedTime.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        for (int i = 0; i < 4; i++) {
            subToSample = new PathContainer.PathContainerBuilder()
                    .setIncrement(0.1)
                    .addCurve(new BezierCurve(
                                    new Vector2D[]{
                                            new Vector2D(0, 32),
                                            new Vector2D(49-0.3*i, 20)
                                    }
                            ),
                            new ParametricHeading(0, Math.PI)
                    )
                    .build();

            subToSample2 = new PathContainer.PathContainerBuilder()
                    .setIncrement(0.1)
                    .addCurve(new BezierCurve(
                                    new Vector2D[]{
                                            new Vector2D(49-0.3*i, 20),
                                            new Vector2D(48.5-0.3*i, -9-0.75*i)
                                    }
                            ),
                            new ParametricHeading(Math.PI, Math.PI)
                    )
                    .build();

            sampleToSub = new PathContainer.PathContainerBuilder()
                    .setIncrement(0.1)
                    .addCurve(new BezierCurve(
                                    new Vector2D[]{
                                            new Vector2D(48.5-0.3*i, -9-0.75*i),
                                            new Vector2D(0, 32.05)
                                    }
                            ),
                            new ParametricHeading(Math.PI, 0)
                    )
                    .build();



            pathFollower = generatePathFollower(subToSample, mecanumController, tracker)
                    .setTimeAfterDeceleration(0.75)
                    .build();

            while (opModeIsActive() && !pathFollower.isCompleted()) {
                loopOpMode();
            }

            pathFollower = generatePathFollower(subToSample2, mecanumController, tracker)
                    .addFinalAction(grabSpecimen)
                    .setTimeAfterDeceleration(0.75)
                    .setSpeed(0.5)
                    .build();

            while (opModeIsActive() && !pathFollower.isCompleted()) {
                loopOpMode();
            }

            pathFollower = generatePathFollower(sampleToSub, mecanumController, tracker)
                    .addFinalAction(placeSpecimen)
                    .setTimeAfterDeceleration(0.75)
                    .build();

            while (opModeIsActive() && !pathFollower.isCompleted()) {
                loopOpMode();
            }
        }
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
        elapsedTime.reset();
        telemetry.update();
    }
}
