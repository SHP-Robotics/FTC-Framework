package org.firstinspires.ftc.teamcode.vision;

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

public class ElementDetectionPipelineRed extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    public static double strictLowS = 140; //TODO: Tune in dashboard
    public static double strictHighS = 255;
    Telemetry telemetry;
    public double leftValue;
    public double rightValue;

    public ElementDetectionPipelineRed() {
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
            new Point(0, 1), //TODO: MAGIC NUMBERS ;-;
            //new Point(399, 447)
            new Point(150, 447)
    );

    static final Rect RIGHT_ROI = new Rect(
            //new Point(400, 1),
            new Point(400, 150),
            new Point(720, 400)
    );

    //threshold(lowest possible) percentage of that color
    //change to 0.4 if too many inconsistencies
    static double THRESHOLD = 0.25; //TODO threshold

    //check if we really need this

    @Override
    public Mat processFrame(Mat input){
        Mat mat = new Mat();

        //From RGB to HSV to for better tuning
        //EasyOpenCV hue is 0-180
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Lower and upper bounds for the color to detect
        Scalar lowHSV = new Scalar(0, 50, 70); //TODO: currently for RED. need to tune
        Scalar highHSV = new Scalar(15, 255, 255); //bruh red is like 0-15 & 350-360 wtf

        Mat detected = new Mat();
        Core.inRange(mat, lowHSV, highHSV, detected); //ONLY returns the pixels in the HSV range

        Mat masked = new Mat();
        //colors the white portion of detected in with the color and outputs to masked
        Core.bitwise_and(mat, mat, masked, detected);
        Scalar average = Core.mean(masked, detected);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();

        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //
        //
        Mat left = scaledThresh.submat(LEFT_ROI);
        Mat right = scaledThresh.submat(RIGHT_ROI);

        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 225;
        rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 225;

        boolean stoneLeft = leftValue > THRESHOLD;
        boolean stoneRight = rightValue > THRESHOLD;

        //        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorExists = new Scalar (0, 255, 0);
        Scalar colorInexistant = new Scalar (255, 0, 0);

        Imgproc.rectangle(scaledThresh, LEFT_ROI, leftValue>rightValue? colorInexistant:colorExists, 3);
        Imgproc.rectangle(scaledThresh, RIGHT_ROI, leftValue<rightValue? colorInexistant:colorExists, 3);
        //

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        //RELEASE EVERYTHING
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        detected.release();
        finalMask.release();
        hierarchy.release();
        left.release();
        right.release();

        return input;

    }
    public int getLocation(){
        if(leftValue>rightValue && leftValue>0.05){
            return 1;
        }
        if(rightValue>leftValue && rightValue>0.05){
            return 2;
        }
        return 3;
    }
}