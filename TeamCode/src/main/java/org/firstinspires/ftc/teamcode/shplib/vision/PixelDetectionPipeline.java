package org.firstinspires.ftc.teamcode.shplib.vision;

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

    CopyOnWriteArrayList<double[]> lastObjects;
    CopyOnWriteArrayList<double[]> objects;

    int patience = 0;
    int breakingPoint = 5;

    public void setPipelineMode(PipelineMode pipelineMode) {
        this.pipelineMode = pipelineMode;
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

    public final double THRESHOLD = 0.015;

    // return pixelMass if concurrentModification = true
    // else return currentPixelMass
    private double pixelMassReadable = 0;
    private boolean concurrentModification = false;

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

        if (!(objects == null || objects.size() == 0) || patience > breakingPoint) {
            lastObjects = objects;
            patience = 0;
        } else {
            patience++;
        }

        objects = new CopyOnWriteArrayList<>();

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(detected, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double pixelMass = 0;

        for (MatOfPoint contour: contours) {
            Moments moments = Imgproc.moments(contour);
            double area = moments.m00/(contour.cols()*contour.rows()*255);
            pixelMass += area;
            if (area > THRESHOLD) {
                objects.add(getObjectFromContour(moments, area));
            }
        }

        concurrentModification = true;
        pixelMassReadable = pixelMass;
        concurrentModification = false;

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        //RELEASE EVERYTHING
        input.release();
        mat.release();
        i1.release();
        i2.release();
        whiteDetected.release();
        greenDetected.release();
        yellowDetected.release();
        purpleDetected.release();
        detected.copyTo(input);
        detected.release();

        return input;
    }

    public double getPixelMass() {
        // wait in line
        while (concurrentModification) {}
        return pixelMassReadable;
    }

    private double[] getObjectFromContour(Moments moments, double area) {
        if (moments.m00 != 0) {
            return new double[]{
                    (moments.m10/moments.m00),
                    (moments.m01/moments.m00),
                    area
            };
        }

        return null;
    }

    public CopyOnWriteArrayList<double[]> getLastObjects() {
        return this.lastObjects;
    }

    public CopyOnWriteArrayList<double[]> getObjects() {
        return this.objects;
    }
}