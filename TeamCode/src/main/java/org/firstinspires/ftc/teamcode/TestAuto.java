package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

@Autonomous
public class TestAuto extends LinearOpMode {
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);

        PathContainer pathContainer = new PathContainer.PathContainerBuilder(new Pose2D(0, 0, 0))
                .addCurve(new BezierCurve(new Vector2D[]{
                        new Vector2D(0, 0),
                        new Vector2D(10, 0)
                }))
                .setIncrement(0.1)
                .build();

        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                pathContainer,
                0.6,
                PestoFTCConfig.DECELERATION,
                new PID(10, 1.5, 0),
                new PID(0.8, 0, 0)
        ).build();

        pathFollower.reset();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pathFollower.update();
            tracker.updateOdometry();

            Vector2D currentPosition = tracker.getCurrentPosition();
            Pose2D currentVelocity = tracker.getRobotVelocity();
            telemetry.addData("X", currentPosition.getX());
            telemetry.addData("Y", currentPosition.getY());
            telemetry.addData("Rotation", tracker.getCurrentHeading());
            telemetry.addLine();
            telemetry.addData("X velocity", currentVelocity.getX());
            telemetry.addData("Y velocity", currentVelocity.getY());
            telemetry.addData("Rotational velocity", currentVelocity.getHeadingRadians());
            telemetry.update();
        }
    }
}
