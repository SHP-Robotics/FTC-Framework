package org.firstinspires.ftc.teamcode.shplib.vision;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class PixelDetectionPipeline extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    public static Scalar lowWhite = new Scalar(0, 0, 170);
    public static Scalar highWhite = new Scalar(180, 30, 255);

    public static Scalar lowGreen = new Scalar(42, 105, 108);
    public static Scalar highGreen = new Scalar(255, 255, 255);

    public static Scalar lowYellow = new Scalar(0, 108, 102);
    public static Scalar highYellow = new Scalar(36, 255, 255);

    public static Scalar lowPurple = new Scalar(120, 48, 110);
    public static Scalar highPurple = new Scalar(255, 255, 255);

    static final Rect LEFT_RECT = new Rect(
            new Point(1, 1), //TODO: MAGIC NUMBERS ;-;
            new Point(219, 448)
    );

    static final Rect CENTER_RECT = new Rect(
            new Point(220, 170),
            new Point(620, 348)
    );

    static final Rect RIGHT_RECT = new Rect(
            new Point(621, 1),
            new Point(800, 448)
    );

    public enum PixelMassLocation {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    private double leftValue;
    private double centerValue;
    private double rightValue;

    public double valueTotal;

//    private double valueWhite;
//    private double valueGreen;
//    private double valuePurple;
//    private double valueYellow;

    final double THRESHOLD = 0.015;

    public PixelDetectionPipeline() {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input){
        Mat mat = new Mat();

        //From RGB to HSV to for better tuning
        //EasyOpenCV hue is 0-180
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat whiteDetected = new Mat();
        Core.inRange(mat, lowWhite, highWhite, whiteDetected); //ONLY returns the white pixels

        Mat greenDetected = new Mat();
        Core.inRange(mat, lowGreen, highGreen, greenDetected); //ONLY returns the colored pixels

        Mat purpleDetected = new Mat();
        Core.inRange(mat, lowPurple, highPurple, purpleDetected); //ONLY returns the colored pixels

        Mat yellowDetected = new Mat();
        Core.inRange(mat, lowYellow, highYellow, yellowDetected); //ONLY returns the colored pixels

        Mat detected = new Mat();

        Mat i1 = new Mat();
        Core.bitwise_or(whiteDetected, greenDetected, i1);

        Mat i2 = new Mat();
        Core.bitwise_or(yellowDetected, purpleDetected, i2);

        Core.bitwise_or(i1, i2, detected);

        Mat left = detected.submat(LEFT_RECT);
        Mat center = detected.submat(CENTER_RECT);
        Mat right = detected.submat(RIGHT_RECT);

        leftValue = Core.sumElems(left).val[0];
        centerValue = Core.sumElems(center).val[0];
        rightValue = Core.sumElems(right).val[0];

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        Scalar colorExists = new Scalar (0, 255, 0);
        Scalar colorInexistant = new Scalar (255, 0, 0);

        Imgproc.rectangle(detected, LEFT_RECT, (leftValue>centerValue && leftValue>rightValue)? colorInexistant:colorExists, 3);
        Imgproc.rectangle(detected, CENTER_RECT, (centerValue>leftValue && centerValue>rightValue)? colorInexistant:colorExists, 3);
        Imgproc.rectangle(detected, RIGHT_RECT, (rightValue>leftValue && rightValue>centerValue)? colorInexistant:colorExists, 3);

        valueTotal = (leftValue+centerValue+rightValue)/(255*(left.cols()+center.cols()+right.cols())*(left.rows()+center.rows()+right.rows()));

        //RELEASE EVERYTHING
        input.release();
        mat.release();
        i1.release();
        i2.release();
//        whiteDetected.copyTo(input);
        whiteDetected.release();
//        greenDetected.copyTo(input);
        greenDetected.release();
//        yellowDetected.copyTo(input);
        yellowDetected.release();
//        purpleDetected.copyTo(input);
        purpleDetected.release();
        detected.copyTo(input);
        detected.release();
        left.release();
        center.release();
        right.release();

        return input;

    }

    public double getPixelMass() {
        if (valueTotal < THRESHOLD) {
            return 0.0;
        }

        return valueTotal;
    }

    public PixelMassLocation getPixelMassLocation() {
        if (valueTotal < THRESHOLD) {
            return PixelMassLocation.NONE;
        }

        if (leftValue > rightValue && leftValue > centerValue) {
            return PixelMassLocation.LEFT;
        }

        if (rightValue > leftValue && rightValue > centerValue) {
            return PixelMassLocation.RIGHT;
        }

        return PixelMassLocation.CENTER;
    }
}