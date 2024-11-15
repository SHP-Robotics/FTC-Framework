package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.shprobotics.pestocore.algorithms.KalmanFilter;

import org.ejml.data.DMatrixRMaj;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Ultrasonic")
public class Ultrasonic extends LinearOpMode {
    AnalogInput distanceSensor1;
    AnalogInput distanceSensor2;

    public static double dF = 1;
    public static double dQ = 1;
    public static double dH = 1;

    public static double dR = 1;

    public static double dFF = 3.3/1.782;

    public double ff(double x) {
        return x * dFF;
    }

    @Override
    public void runOpMode() {
        distanceSensor1 = hardwareMap.get(AnalogInput.class, "ultra1");
        distanceSensor2 = hardwareMap.get(AnalogInput.class, "ultra2");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        KalmanFilter kalmanFilter = new KalmanFilter();

        DMatrixRMaj F = new DMatrixRMaj(new double[][]{
                {dF},
        });

        DMatrixRMaj Q = new DMatrixRMaj(new double[][]{
                {dQ},
        });

        DMatrixRMaj H = new DMatrixRMaj(new double[][]{
                {dH},
        });

        kalmanFilter.configure(F, Q, H);

        DMatrixRMaj z = new DMatrixRMaj(new double[][]{
                {0},
        });

        DMatrixRMaj R = new DMatrixRMaj(new double[][]{
                {dR},
        });

        kalmanFilter.setState(z, R);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            z = new DMatrixRMaj(new double[][]{
                    {ff(distanceSensor1.getVoltage())},
            });

            kalmanFilter.update(z, R);
            kalmanFilter.getState().get(0, 0);

            telemetry.addData("U1, Voltage", distanceSensor1.getVoltage());
            telemetry.addData("U1, Kalman", kalmanFilter.getState().get(0, 0));
            telemetry.addData("U1, Meters", ff(distanceSensor1.getVoltage()));

            telemetry.addLine();

            telemetry.addData("U2, Voltage", distanceSensor2.getVoltage());
            telemetry.addData("U2, Meters", ff(distanceSensor2.getVoltage()));

            telemetry.update();

            dashboardTelemetry.addData("Voltage", ff(distanceSensor1.getVoltage()));
            dashboardTelemetry.addData("Kalman", kalmanFilter.getState().get(0, 0));

            dashboardTelemetry.update();
        }
    }
}
