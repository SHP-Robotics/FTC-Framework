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

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        DeterministicTracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        ViperSlideSubsystem viperSlideSubsystem;
        WormGearSubsystem wormGearSubsystem;
        wormGearSubsystem = new WormGearSubsystem(hardwareMap);
        viperSlideSubsystem = new ViperSlideSubsystem(hardwareMap);

        PathContainer pathContainer = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(
                        new Vector2D[]{
                                new Vector2D(0,0),
                                new Vector2D(1,5),
                                new Vector2D(-1,10),
                        }
                ))
//                .addCurve(new BezierCurve(
//                        new Vector2D[]{
//                                new Vector2D(-1,10),
//                                new Vector2D(-1.1,10.25),
//                                new Vector2D(0,15),
//                                new Vector2D(-10,15),
//                        }
//                ))
                .setIncrement(0.01).build();
        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                pathContainer

        )
                .setDeceleration(0)
                .setEndpointPID(new PID(0, 0, 0))
                .setHeadingPID(new PID(0, 0, 0))
                .setSpeed(1)
                .build();

        pathFollower.reset(); // Reset once, before the loop

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("position",tracker.getCurrentPosition());
            telemetry.addData("velocity",tracker.getRobotVelocity());
            telemetry.addData("completed",  pathFollower.isCompleted());
            telemetry.addData("Path Progress", pathFollower.isCompleted() ? "Completed" : "In Progress");
            telemetry.update();
            if (gamepad1.x) {
                gamepad1.rumble(1000);
                pathFollower.reset(); // Reset once, before the loop
                tracker.update();

                pathFollower.update();
            }

        }
    }
}
