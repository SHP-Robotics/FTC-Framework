package org.firstinspires.ftc.teamcode;

public class Matrices {
    public static native double[][] multiply(double[][] matrix1, double[][] matrix2);
    public static native double[] add(double[] matrix1, double[] matrix2);
    public static native double[] relu(double[] matrix1);

    static {
       System.loadLibrary("Matrices");
    }
}
