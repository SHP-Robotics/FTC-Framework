package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

@Autonomous
public class DontPressSquareAutonomous extends LinearOpMode {
    MecanumController mecanumController;
    DeterministicTracker tracker;
    PathFollower pathFollower;
    ViperSlideSubsystem viperSlideSubsystem;
    WormGearSubsystem wormGearSubsystem;
    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(0.04,0, 0))
                .setHeadingPID(new PID(0.8, 0, 0))
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setSpeed(1)
                .setEndTolerance(0.4, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(4)
                .build();


    }

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);


        wormGearSubsystem = new WormGearSubsystem(hardwareMap);
        viperSlideSubsystem = new ViperSlideSubsystem(hardwareMap);

        PathContainer Intake1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(

                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(-32, 57),
                                }
                        )
                )
                .build();
        PathContainer Intake2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(

                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(-32, 57),
                                }
                        )
                )
                .build();


        waitForStart();
        tracker.reset();

        followPath(Intake1, 2, 1);
        cycle();
        followPath(Intake2, 2, 1);
        cycle();


    }

    public void loopOpMode () {
        telemetry.addData("position", tracker.getCurrentPosition());
        telemetry.addData("velocity", tracker.getRobotVelocity());
        telemetry.addData("Path Progress", pathFollower.isCompleted() ? "Completed" : "In Progress");
        tracker.update();
        pathFollower.update();
        telemetry.update();
    }

    public void followPath (PathContainer path,double deceleration, double speed){
        if (isStopRequested()) return;
        pathFollower = generatePathFollower(path, deceleration, speed);

        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }
    /**
     * make functions out here
     */
    public void cycle(){
        wormGearSubsystem.update();
        viperSlideSubsystem.update();
        wormGearSubsystem.cycle();
        viperSlideSubsystem.cycle();

    }


}



