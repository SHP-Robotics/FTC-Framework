package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@TeleOp()
public class PixelClassifier extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor testSensor = (ColorSensor) hardwareMap.get("colorSensor");

        double[][] weights = new double[][]{
                new double[]{0.26554176, -0.21040677, 0.15837221, -0.0598264, 0.05147775},
                new double[]{-0.09597304, 0.01706841, 0.00319473, -0.05055667, 0.3494639},
                new double[]{-0.27730775, -0.08587321, -0.32321468, -0.11201918, -1.1045761}
        };

        double[] bias = new double[]{2.6694434, -1.206498, -0.56916445, -0.21354164, -0.71332103};

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double r = testSensor.red();
            double g = testSensor.green();
            double b = testSensor.blue();

            double[] result = new double[]{0, 0, 0, 0, 0};
            int argmax = 0;
            double max = 0;

            for (int i = 0; i < 5; i++) {
                result[i] = (r * weights[0][i]) + (g * weights[1][i]) + (b * weights[2][i]) + bias[i];
            }

            for (int i = 0; i < 5; i++) {
                if (result[i] <= max) {
                    argmax = i;
                    max = result[i];
                }
            }

            switch (argmax) {
                case 0:
                    telemetry.addLine("nothing");
                    break;
                case 1:
                    telemetry.addLine("white");
                    break;
                case 2:
                    telemetry.addLine("green");
                    break;
                case 3:
                    telemetry.addLine("purple");
                    break;
                case 4:
                    telemetry.addLine("yellow");
                    break;
            }

            telemetry.update();
            sleep(5);
        }
    }
}
