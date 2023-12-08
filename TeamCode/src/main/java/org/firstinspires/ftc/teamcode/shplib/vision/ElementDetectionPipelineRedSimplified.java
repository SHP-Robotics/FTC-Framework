package org.firstinspires.ftc.teamcode.shplib.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ElementDetectionPipelineRedSimplified extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    private Scalar lowPixel = new Scalar(200, 200, 200);
    private Scalar highPixel = new Scalar(255, 255, 255);

    Telemetry telemetry;

    public double leftValue;
    public double rightValue;

    public ElementDetectionPipelineRedSimplified() {
        frameList = new ArrayList<>();
    }

    public enum loc{
        RIGHT,
        LEFT,
        NONE
    };
    //loc location;
    //sub matrices to divide image
    //we can draw rectangles on the screen to find the perfect fit
    //edit as necessary
    static final Rect LEFT_ROI = new Rect(
            new Point(1, 1), //TODO: MAGIC NUMBERS ;-;
            new Point(210, 447)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point(400, 10),
            new Point(700, 447)
    );

    //threshold(lowest possible) percentage of that color
    static double THRESHOLD = 0.4; //TODO threshold

    public double getMatValue(Scalar pixelLow, Scalar pixelHigh, Mat mat) {
        Mat detected = new Mat();
        Core.inRange(mat, pixelLow, pixelHigh, detected); //ONLY returns the pixels in the range

        return (double) Core.countNonZero(detected) / (detected.rows() * detected.cols());
    }

    @Override
    public Mat processFrame(Mat input){
        Mat hsv = new Mat();

        hsv.copyTo(input);

        Mat left = hsv.submat(LEFT_ROI);
        Mat right = hsv.submat(RIGHT_ROI);

        leftValue = getMatValue(lowPixel, highPixel, left);
        rightValue = getMatValue(lowPixel, highPixel, right);

        //RELEASE EVERYTHING
        input.release();
        left.copyTo(input);
        hsv.release();
        left.release();
        right.release();

        return input;

    }
    public int getLocation(){
        if(leftValue>rightValue/* && leftValue>THRESHOLD*/){
            return 1;
        }
        if(rightValue>leftValue/* && rightValue>THRESHOLD*/){
            return 2;
        }
        return 3;

    }
}