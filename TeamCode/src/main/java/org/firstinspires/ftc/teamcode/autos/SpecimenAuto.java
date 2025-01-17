package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SpecimenSubsystem;

@Config
@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;

    FourBarSubsystem fourBarSubsystem;

    SpecimenSubsystem specimenSubsystem;
    ClawSubsystem clawSubsystem;

    PathContainer depositSpecimen1;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;

    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(PestoFTCConfig.endpointPID)
                .setHeadingPID(PestoFTCConfig.headingPID)
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setSpeed(speed)

//                .setEndTolerance(0.4, Math.toRadians(0))
//                .setEndVelocityTolerance(4)
//                .setTimeAfterDeceleration(deceleration)
                .build();
    }

    @Override
    public void runOpMode() {
        depositSpecimen1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.03)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(0, 29)
                                }
                        )
                )
                .build();

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);

        fourBarSubsystem = new FourBarSubsystem(hardwareMap);

        specimenSubsystem = new SpecimenSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);

        //TODO THIS IS THE START

        waitForStart();

        //Prepare to deposit preload
        specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.HIGH);
        specimenSubsystem.update();

        //Deposit Specimen 1
        followPath(depositSpecimen1, 1, 0.6);

        specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.DEPOSIT_HIGH);
        specimenSubsystem.update();

        timeout(0.5);

        clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);
        clawSubsystem.update();

        timeout(0.5);
    }

    public void loopOpMode() {
        tracker.update();
        pathFollower.update();

        fourBarSubsystem.update();

        specimenSubsystem.update();
        clawSubsystem.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        elapsedTime.reset();
        telemetry.update();
    }

    public void followPath(PathContainer path, double deceleration, double speed) {
        if (isStopRequested()) return;

        pathFollower = generatePathFollower(path, deceleration, speed);

        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }

    public void timeout(double time) {
        elapsedTime.reset();

        while (elapsedTime.seconds() < time)
            tracker.update();
    }
}