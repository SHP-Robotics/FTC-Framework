package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.ExponentialSmoothingFilter;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class GeometryTest extends LinearOpMode {
    public static double alpha = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
//        BezierCurve bezierCurve = new BezierCurve(new Vector2D[]{
//                new Vector2D(10, 10),
//                new Vector2D(10, 0),
//                new Vector2D(0, 0),
//                new Vector2D(0, -10)
//        });
//
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            double stick = Math.max(0, -gamepad1.left_stick_y);
//            bezierCurve.increment(stick - bezierCurve.getT());
//            Vector2D current = bezierCurve.getPoint();
//            telemetry.addData("T", stick);
//            telemetry.addLine();
//            telemetry.addData("X", current.getX());
//            telemetry.addData("Y", current.getY());
//            telemetry.update();
//        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        double lastTime = elapsedTime.seconds();

        ExponentialSmoothingFilter exponentialSmoothingFilter = new ExponentialSmoothingFilter(alpha);

        dashboardTelemetry.addData("gamepad-left-stick-y", 0);
        dashboardTelemetry.addData("filtered-gamepad-left-stick-y", 0);
        dashboardTelemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            dashboardTelemetry.addData("gamepad-left-stick-y", -gamepad1.left_stick_y);
            dashboardTelemetry.addData("filtered-gamepad-left-stick-y", exponentialSmoothingFilter.getOutput(-gamepad1.left_stick_y, elapsedTime.seconds() - lastTime));
            lastTime = elapsedTime.seconds();
            dashboardTelemetry.update();
        }
    }
}
