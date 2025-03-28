package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "matmul")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        double[][] matrix1 = new double[][]{
                new double[]{-1, -1, 4},
                new double[]{0, 3, -3},
                new double[]{2, -1, -2},
        };

        double[][] matrix2 = new double[][]{
                new double[]{3, 2, 3},
                new double[]{2, 2, 1},
                new double[]{2, 1, 1},
        };


        double start = System.nanoTime();
        double[][] out = Matrices.multiply(matrix1, matrix2);
        double end = System.nanoTime();

        telemetry.addData("time", (end - start) / (1E9));

        for (int i = 0; i < out.length; i++) {
            for (int j = 0; j < out[0].length; j++) {
                telemetry.addData("i " + i + " j " + j, out[i][j]);
            }
        }

        telemetry.update();

        waitForStart();
    }
}
