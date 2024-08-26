package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.KalmanFilter;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.ejml.data.DMatrixRMaj;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@TeleOp
public class RobotCentricDrive extends LinearOpMode {
    public void runOpMode() {
        // F = X, X
        // Q = X, X
        // H = Z, X

        // x = X, X
        // P = X, X

        // z = Z, X
        // R = Z, Z

        // F, the state-transition model;
        // H, the observation model;
        // Q, the covariance of the process noise;
        // R, the covariance of the observation noise;


        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        ColorSensor clawColorSensor = hardwareMap.get(ColorSensor.class, "clawColor");
        ColorSensor backColorSensor = hardwareMap.get(ColorSensor.class, "backColor");

        Servo clawServo = hardwareMap.get(Servo.class, "claw");

        double close = 0.16;
        double open = 0.25;
        boolean seen = false;


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        KalmanFilter kalmanFilter = new KalmanFilter();

        DMatrixRMaj F = new DMatrixRMaj(new double[][]{
                {1, 1},
                {1, 1}
        });

        DMatrixRMaj Q = new DMatrixRMaj(new double[][]{
                {0, 0},
                {0, 0}
        });

        DMatrixRMaj H = new DMatrixRMaj(new double[][]{
                {1, 1},
                {1, 1}
        });

        DMatrixRMaj R = new DMatrixRMaj(new double[][]{
                {0, 0},
                {0, 0}
        });

        kalmanFilter.configure(F, Q, H);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double start = timer.time();

        while (opModeIsActive() && !isStopRequested()) {
            tracker.updateOdometry();
            teleOpController.updateSpeed(gamepad1);
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if ((!seen && clawColorSensor.alpha() > 2000) || gamepad1.right_bumper) {
                clawServo.setPosition(close);
            } else if (clawColorSensor.alpha() < 2000 || gamepad1.left_bumper) {
                clawServo.setPosition(open);
            }

            seen = clawColorSensor.alpha() > 2000;

            Vector2D robotPosition = tracker.getCurrentPosition();
            telemetry.addData("x", robotPosition.getX());
            telemetry.addData("y", robotPosition.getY());
            telemetry.addData("r", tracker.getCurrentHeading());

            telemetry.addLine();

            telemetry.addData("clawColorSensor r", clawColorSensor.red());
            telemetry.addData("clawColorSensor g", clawColorSensor.green());
            telemetry.addData("clawColorSensor b", clawColorSensor.blue());
            telemetry.addData("clawColorSensor a", clawColorSensor.alpha());

            telemetry.addLine();

            telemetry.addData("backColorSensor r", backColorSensor.red());
            telemetry.addData("backColorSensor g", backColorSensor.green());
            telemetry.addData("backColorSensor b", backColorSensor.blue());
            telemetry.addData("backColorSensor a", backColorSensor.alpha());

            telemetry.addData("Time", timer.time() - start);
            start = timer.time();

            dashboardTelemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());

            DMatrixRMaj z = new DMatrixRMaj(new double[][]{
                    {1}
            });

            kalmanFilter.update(z, R);

            dashboardTelemetry.addData("Kalman", kalmanFilter.getState().data);
            dashboardTelemetry.update();

            telemetry.update();
        }
    }
}
