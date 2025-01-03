package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@Autonomous(name = "Red Auto")
public class TestAuto extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horizontal;
    ClawSubsystem claw;
    PathContainer startToSub, depositToSub, subToBlock1, subToBlock2, subToBlock3, subToBlock4;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;

    public PathFollower generatePathFollower(PathContainer pathContainer, Runnable finalAction, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(0.002,0, 0))
                .setHeadingPID(new PID(1.0, 0, 0))
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setSpeed(speed)
//                .setDecelerationFunction(PathFollower.SQUID_DECELERATION)
                //^^^ combats static friction
                // takes the square root of PID. PID controls drive speed
                // call this "SQUID"
                //.setCheckFinishedFunction()
                .setEndTolerance(0.4, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(deceleration)
                .addFinalAction(finalAction)
                .build();
    }

    @Override
    public void runOpMode() {
        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        vertical = new VerticalSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        rotate = new RotateSubsystem(hardwareMap);
        horizontal = new HorizSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        claw.close();
        pivot.processState(PivotSubsystem.State.DRIVING);

        waitForStart();

        //drop off preload
        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(4, -29)
                                }
                        )
                )
                .build();

        pathFollower = generatePathFollower(startToSub, () -> {
            tracker.update();
        }, 1.0, 0.6);

//        waitForStart();

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        horizontal.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.OUTTAKE1);
        vertical.setDepositState(VerticalSubsystem.State.HIGHBAR);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        updateCommands(0.5);
        pivot.setState(PivotSubsystem.State.OUTTAKE2);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        updateCommands(1);

        depositToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(4, -29),
                                        new Vector2D(4, -32)
                                }
                        )
                )
                .build();

        pathFollower = generatePathFollower(depositToSub, () -> {
            tracker.update();
        }, 1.0, 0.6);

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
        pivot.setState(PivotSubsystem.State.OUTTAKE3);
        vertical.setState(VerticalSubsystem.State.DOWN);
        updateCommands(0.5);
        claw.open();
        updateCommands(0.5);
        claw.close();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        pivot.setState(PivotSubsystem.State.DRIVING);
        updateCommands(1); //TODO test to make smaller

        //drive to block
        subToBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(4, -30),
                                        new Vector2D(-37, -15) //-40
                                }
                        )
                )
                .build();

        pathFollower = generatePathFollower(subToBlock1, () -> {
            tracker.update();
        }, 1.0, 0.6);

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        subToBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-37, -15),
                                        new Vector2D(-37, -46)
                                }
                        )
                )
                .build();

        pathFollower = generatePathFollower(subToBlock2, () -> {
            tracker.update();
        }, 1.0, 0.6);

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        subToBlock3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-37, -46),
                                        new Vector2D(-42, -46)
                                }
                        )
                )
                .build();

        pathFollower = generatePathFollower(subToBlock3, () -> {
            tracker.update();
        }, 1.0, 0.6);

        while (opModeIsActive() && !pathFollower.isCompleted()) {
            loopOpMode();
        }

        subToBlock4 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-42, -46),
                                        new Vector2D(-42, -5),
                                        new Vector2D(-42, -5),
                                        new Vector2D(-42, -5)
                                }
                        )
                )
                .build();

        pathFollower = generatePathFollower(subToBlock4, () -> {
            tracker.update();
        }, 0.2, 0.6);

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
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        tracker.update();

        pathFollower.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        telemetry.addData("fini", startToSub.isFinished());
//        telemetry.addData("ex", subToSample.getEndpoint().getX());
//        telemetry.addData("ey", subToSample.getEndpoint().getY());
        elapsedTime.reset();
        telemetry.update();
    }

    public void updateCommands(double seconds){
        elapsedTime.reset();
        while (elapsedTime.seconds() < seconds)
            try {
                CommandScheduler.getInstance().run();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
    }

    public void update(){
    }
}