package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.BELOW_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "One Specimen")
public class OneSpecimen extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    PathContainer startToSub;
    PathContainer subToSample;
    PathContainer subToSample2;
    PathContainer sampleToSub;

    PathFollower pathFollower;

    private SpecimenClawSubsystem clawSubsystem;
    private SlideSubsystem slideSubsystem;

    private ElapsedTime elapsedTime;

    public PathFollower generatePathFollower(PathContainer pathContainer, Runnable finalAction, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(0.004,0, 0))
                .setHeadingPID(new PID(0.4, 0, 0))
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
                                new Vector2D(0, -32)
                        }
                ), new ParametricHeading(0, 0))
                .build();

        pathFollower = generatePathFollower(startToSub, () -> {
            elapsedTime.reset();
            while (opModeIsActive() && elapsedTime.seconds() < 0.5)
                slideSubsystem.update();
            slideSubsystem.setState(BELOW_HIGH_RUNG);
            elapsedTime.reset();
            while (opModeIsActive() && elapsedTime.seconds() < 2.0)
                slideSubsystem.update();

            elapsedTime.reset();
            clawSubsystem.setState(SpecimenClawSubsystem.ClawState.OPEN);

            while (opModeIsActive() && elapsedTime.seconds() < 0.5)
                clawSubsystem.update();

            slideSubsystem.setState(INTAKE);
        }, 1.0, 0.6);

        clawSubsystem = new SpecimenClawSubsystem(hardwareMap);
        clawSubsystem.setState(SpecimenClawSubsystem.ClawState.CLOSE);
        clawSubsystem.update();
        slideSubsystem = new SlideSubsystem(hardwareMap);

        elapsedTime = new ElapsedTime();

        waitForStart();
        slideSubsystem.init();
        slideSubsystem.setState(ABOVE_HIGH_RUNG);

        elapsedTime.reset();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        subToSample = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, -32),
                                        new Vector2D(0, -10)
                                }
                        ), new ParametricHeading(0, 0)
                )
                .build();

        pathFollower = generatePathFollower(subToSample, () -> {}, 0.75, 0.6);

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        subToSample2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(-50, 0)
                                }
                        ), new ParametricHeading(0, 0)
                )
                .build();

        pathFollower = generatePathFollower(subToSample2, () -> {}, 0.75, 0.6);

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        slideSubsystem.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideSubsystem.setPower(-0.3);
        sleep(500);
        slideSubsystem.setPower(0);
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
