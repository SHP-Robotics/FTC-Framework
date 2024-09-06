package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.shprobotics.pestocore.algorithms.KalmanFilter;

import org.ejml.data.DMatrixRMaj;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Ultrasonic")
public class Ultrasonic extends LinearOpMode {
    AnalogInput distanceSensor1;
    AnalogInput distanceSensor2;

    public double ff(double x) {
        return x * 3.3;
    }

    @Override
    public void runOpMode() {
        distanceSensor1 = hardwareMap.get(AnalogInput.class, "ultra1");
        distanceSensor2 = hardwareMap.get(AnalogInput.class, "ultra2");

        KalmanFilter kalmanFilter = new KalmanFilter();

        DMatrixRMaj F = new DMatrixRMaj(new double[][]{
                {1, 0},
                {0, 1}
        });

        DMatrixRMaj Q = new DMatrixRMaj(new double[][]{
                {0.1, 0},
                {0, 0.1}
        });

        DMatrixRMaj H = new DMatrixRMaj(new double[][]{
                {1, 0},
                {0, 1}
        });

        DMatrixRMaj R = new DMatrixRMaj(new double[][]{
                {0.25, 0},
                {0, 0.25}
        });

        kalmanFilter.configure(F, Q, H);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("U1, Voltage", distanceSensor1.getVoltage());
            telemetry.addData("U1, Meters", ff(distanceSensor1.getVoltage()));

            telemetry.addLine();

            DMatrixRMaj z = new DMatrixRMaj(new double[][]{
                    {ff(distanceSensor1.getVoltage())}
            });

            kalmanFilter.update(z, R);
            
            telemetry.addData("U2, Voltage", distanceSensor2.getVoltage());
            telemetry.addData("U2, YHat Meters", kalmanFilter.getState());
            telemetry.addData("U2, Meters", ff(distanceSensor2.getVoltage()));

            telemetry.update();

            dashboardTelemetry.addData("Voltage", ff(distanceSensor2.getVoltage()));

            dashboardTelemetry.addData("Kalman", kalmanFilter.getState().data);
            dashboardTelemetry.update();
        }
    }
}
