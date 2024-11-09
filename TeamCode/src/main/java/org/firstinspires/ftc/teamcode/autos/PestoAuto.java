package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@Config
@Autonomous
public class PestoAuto extends LinearOpMode {
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;

    @Override
    public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        PathContainer path = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(new Vector2D[]{
                        new Vector2D(0, 0),
                        new Vector2D(0, 10)
                }))
                .setIncrement(0.01)
                .build();
        PathFollower follower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                path)
//                0.3,
//                PestoFTCConfig.DECELERATION,
//                new PID(0.1, 0, 0),
//                new PID(0.1, 0, 0))
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setHeadingPID( new PID(0.1, 0, 0))
                .setEndpointPID(new PID(0.1, 0, 0))
                .setSpeed(0.3)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
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
