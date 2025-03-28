package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class FullyConnected {
    public static double[][] run(double[][] inputs) {
        ArrayList<double[][]> weights = new ArrayList<>();

        weights.add(new double[][]{
                new double[]{0, 0, 0},
                new double[]{0, 0, 0},
                new double[]{0, 0, 0},
        });

        ArrayList<double[]> biases = new ArrayList<>();

        biases.add(new double[]{
                0, 0, 0
        });

        int n = weights.size();

        double[][] x = inputs;

        for (int i = 0; i < n; i++) {
            x = Matrices.multiply(x, weights.get(i));
            x[0] = Matrices.add(x[0], biases.get(i));
            x[0] = Matrices.relu(x[0]);
        }

        return x;
    }
}
