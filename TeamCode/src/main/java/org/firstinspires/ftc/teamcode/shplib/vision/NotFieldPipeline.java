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

public class NotFieldPipeline extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    public double value;

    public NotFieldPipeline() {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input){
        Mat mat = new Mat();

        //From RGB to HSV to for better tuning
        //EasyOpenCV hue is 0-180
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowWhite = new Scalar(0, 0, 170);
        Scalar highWhite = new Scalar(180, 30, 255);

        Scalar lowColor = new Scalar(0, 30, 150);
        Scalar highColor = new Scalar(180, 255, 255);

        Mat whiteDetected = new Mat();
        Core.inRange(mat, lowWhite, highWhite, whiteDetected); //ONLY returns the white pixels

        Mat colorDetected = new Mat();
        Core.inRange(mat, lowColor, highColor, colorDetected); //ONLY returns the colored pixels

        Mat detected = new Mat();
        Core.bitwise_or(whiteDetected, colorDetected, detected);

        value = Core.sumElems(detected).val[0] / (255*detected.cols()*detected.rows());

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        //RELEASE EVERYTHING
        input.release();
        mat.release();
//        whiteDetected.copyTo(input);
        whiteDetected.release();
//        colorDetected.copyTo(input);
        colorDetected.release();
        detected.copyTo(input);
        detected.release();

        return input;

    }

    public double getAmountNotField() {
        return value;
    }
}