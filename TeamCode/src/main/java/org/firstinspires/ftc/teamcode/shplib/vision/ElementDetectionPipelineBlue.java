package org.firstinspires.ftc.teamcode.shplib.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.LocalizationTest;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ElementDetectionPipelineBlue extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    public static double strictLowS = 140; //TODO: Tune in dashboard
    public static double strictHighS = 255;
    Telemetry telemetry;
    public double leftValue;
    public double rightValue;
    public double totalValue;

    public boolean isReadable = true;
    public int maxHeightReadable = 0;

    public ElementDetectionPipelineBlue() {
        frameList = new ArrayList<>();
    }

    public enum LocationPosition {
        LEFT,
        RIGHT,
        NONE
    }

    //sub matrices to divide image
    //we can draw rectangles on the screen to find the perfect fit
    //edit as necessary
    static final Rect LEFT_ROI = new Rect(
            new Point(1, 1), //TODO: MAGIC NUMBERS ;-;
            new Point(200, 447)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point(200, 1),
            new Point(500, 300)
    );

    //threshold(lowest possible) percentage of that color
    static double THRESHOLD = 0.08; //TODO threshold

    //check if we really need this

    @Override
    public Mat processFrame(Mat input){
        Mat mat = new Mat();

        //From RGB to HSV to for better tuning
        //EasyOpenCV hue is 0-180
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Lower and upper bounds for the color to detect
        Scalar lowHSV = new Scalar(210/2, 50, 70); //TODO: currently for RED. need to tune
        Scalar highHSV = new Scalar(250/2, 255, 255); //bruh red is like 0-15 & 350-360 wtf

        Mat detected = new Mat();
        Core.inRange(mat, lowHSV, highHSV, detected); //ONLY returns the pixels in the HSV range

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(detected, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int maxHeight = 0;
        for (MatOfPoint contour: contours) {
            int height = contour.rows();
            if (height > maxHeight) {
                maxHeight = height;
            }
        }
        isReadable = false;
        maxHeightReadable = maxHeight;
        isReadable = true;

        Mat left = detected.submat(LEFT_ROI);
        Mat right = detected.submat(RIGHT_ROI);

        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 225;
        rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 225;
        totalValue = Core.sumElems(detected).val[0] / (detected.rows() * detected.cols() * 225);

        //        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorExists = new Scalar (0, 255, 0);
        Scalar colorInexistant = new Scalar (255, 0, 0);

        Imgproc.rectangle(detected, LEFT_ROI, leftValue>rightValue? colorInexistant:colorExists, 3);
        Imgproc.rectangle(detected, RIGHT_ROI, leftValue<rightValue? colorInexistant:colorExists, 3);
        //

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        //RELEASE EVERYTHING
        input.release();
        mat.release();
        detected.copyTo(input);
        detected.release();
        left.release();
        right.release();

        return input;

    }
    public LocationPosition getLocation(){
        if(leftValue>rightValue && leftValue > THRESHOLD){
            return LocationPosition.LEFT;
        }
        if(rightValue>leftValue && rightValue > THRESHOLD){
            return LocationPosition.RIGHT;
        }
        return LocationPosition.NONE;

    }

    public int getMaxHeightReadable() {
        while (!isReadable) {}
        return maxHeightReadable;
    }
}