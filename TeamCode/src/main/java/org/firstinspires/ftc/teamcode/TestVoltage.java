package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.algorithms.KalmanFilter;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class TestVoltage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        KalmanFilter kalmanFilter = new KalmanFilter();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            dashboardTelemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
            dashboardTelemetry.update();
        }
    }
}
