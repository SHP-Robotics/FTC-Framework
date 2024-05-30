package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class PestoAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

//        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
//        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
//        PathContainer path = new PathContainer.PathContainerBuilder()
//                .addCurve(new BezierCurve(new Vector2D[]{
//                        new Vector2D(0, 0),
//                        new Vector2D(0, 10)
//                }))
//                .setIncrement(0.02)
//                .build();
//        PathFollower follower = new PathFollower.PathFollowerBuilder(
//                mecanumController,
//                tracker,
//                path,
//                0.3,
//                PestoFTCConfig.DECELERATION,
//                new PID(1, 0, 0, elapsedTime),
//                new PID(1, 0, 0, elapsedTime))
//                .build();
//
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            follower.update();
//            tracker.updateOdometry();
//
//            Vector2D currentPosition = tracker.getCurrentPosition();
//            Pose2D currentVelocity = tracker.getRobotVelocity();
//            telemetry.addData("X", currentPosition.getX());
//            telemetry.addData("Y", currentPosition.getY());
//            telemetry.addData("Rotation", tracker.getCurrentHeading());
//            telemetry.addLine();
//            telemetry.addData("X velocity", currentVelocity.getX());
//            telemetry.addData("Y velocity", currentVelocity.getY());
//            telemetry.addData("Rotational velocity", currentVelocity.getHeadingRadians());
//            telemetry.update();
//        }

        PID test = new PID(1, 0, 0, elapsedTime);
        test.reset();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (opModeIsActive() && !isStopRequested()) {
            dashboardTelemetry.addData("Output", test.getOutput(0, -gamepad1.left_stick_y));
            dashboardTelemetry.update();
        }
    }
}
