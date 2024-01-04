package org.firstinspires.ftc.teamcode.shplib.vision;

import static org.opencv.core.Core.max;
import static org.opencv.imgproc.Imgproc.moments;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvTracker;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.concurrent.CopyOnWriteArrayList;

public class PixelDetectionPipeline extends OpenCvPipeline {
    public enum PipelineMode {
        WHITE_ONLY,
        GREEN_ONLY,
        YELLOW_ONLY,
        PURPLE_ONLY,
        ALL_PIXELS
    }

    private PipelineMode pipelineMode = PipelineMode.ALL_PIXELS;
    private boolean trackingCenters = false;

    CopyOnWriteArrayList<double[]> lastObjects;
    CopyOnWriteArrayList<double[]> objects;

    int patience = 0;
    int breakingPoint = 5;

    public void setPipelineMode(PipelineMode pipelineMode) {
        this.pipelineMode = pipelineMode;
    }

    public void setTrackingCenters(boolean trackingCenters) {
        this.trackingCenters = trackingCenters;
    }

    ArrayList<double[]> frameList;

    public static Scalar lowWhite = new Scalar(0, 0, 159);
    public static Scalar highWhite = new Scalar(180, 30, 255);

    public static Scalar lowGreen = new Scalar(42, 105, 108);
    public static Scalar highGreen = new Scalar(57, 219, 222);

    public static Scalar lowYellow = new Scalar(0, 108, 102);
    public static Scalar highYellow = new Scalar(36, 255, 255);

    public static Scalar lowPurple = new Scalar(120, 48, 110);
    public static Scalar highPurple = new Scalar(138, 102, 255);

    static final Rect LEFT_RECT = new Rect(
            new Point(1, 1), //TODO: MAGIC NUMBERS ;-;
            new Point(219, 448)
    );

    static final Rect CENTER_RECT = new Rect(
//            new Point(220, 170),
//            new Point(620, 348)
            new Point(220, 0),
            new Point(620, 448)
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
        objects = new CopyOnWriteArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input){
        Mat mat = new Mat();

        //From RGB to HSV to for better tuning
        //EasyOpenCV hue is 0-180
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat whiteDetected = new Mat();
        Mat greenDetected = new Mat();
        Mat purpleDetected = new Mat();
        Mat yellowDetected = new Mat();

        Mat detected = new Mat();

        Mat i1 = new Mat();
        Mat i2 = new Mat();

        switch (pipelineMode) {
            case WHITE_ONLY:
                Core.inRange(mat, lowWhite, highWhite, detected);
                break;
            case GREEN_ONLY:
                Core.inRange(mat, lowGreen, highGreen, detected);
                break;
            case PURPLE_ONLY:
                Core.inRange(mat, lowPurple, highPurple, detected);
                break;
            case YELLOW_ONLY:
                Core.inRange(mat, lowYellow, highYellow, detected);
                break;
            default:
                Core.inRange(mat, lowWhite, highWhite, whiteDetected); //ONLY returns the white pixels
                Core.inRange(mat, lowGreen, highGreen, greenDetected); //ONLY returns the colored pixels
                Core.inRange(mat, lowPurple, highPurple, purpleDetected); //ONLY returns the colored pixels
                Core.inRange(mat, lowYellow, highYellow, yellowDetected); //ONLY returns the colored pixels

                Core.bitwise_or(whiteDetected, greenDetected, i1);
                Core.bitwise_or(yellowDetected, purpleDetected, i2);
                Core.bitwise_or(i1, i2, detected);
                break;
        }

        if (trackingCenters) {
            if (!(objects == null || objects.size() == 0) || patience > breakingPoint) {
                lastObjects = objects;
                patience = 0;
            } else {
                patience++;
            }

            objects = new CopyOnWriteArrayList<>();

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            //find contours, input scaledThresh because it has hard edges
            Imgproc.findContours(detected, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//            double largestArea = 0;
//            MatOfPoint largestContour = new MatOfPoint();
//            for (MatOfPoint contour: contours) {
//                Moments moments = Imgproc.moments(contour);
//                double area = moments.m00/(contour.cols()*contour.rows()*255);
//                if (moments.m00 >= largestArea && area > THRESHOLD) {
//                    largestContour = contour;
//                    largestArea = area;
//                }
//            }
//
//            if (largestArea > 0) {
//                objects.add(getObject(largestContour, largestArea));
//            }

            for (MatOfPoint contour: contours) {
                Moments moments = Imgproc.moments(contour);
                double area = moments.m00/(contour.cols()*contour.rows()*255);
                if (area > THRESHOLD) {
                    objects.add(getObject(contour, area));
                }
            }
        }

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

    public CopyOnWriteArrayList<double[]> getLastObjects() {
        return this.lastObjects;
    }

    private double[] getObject(MatOfPoint contour, double area) {
        Moments moments = Imgproc.moments(contour);
        if (moments.m00 != 0) {
            return new double[]{
                    (moments.m10/moments.m00),
                    (moments.m01/moments.m00),
                    area
            };
        }

        return null;
    }

    public CopyOnWriteArrayList<double[]> getObjects() {
        return this.objects;
    }
}