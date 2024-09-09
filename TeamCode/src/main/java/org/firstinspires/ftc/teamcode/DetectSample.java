package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.vision.ComputerVision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class DetectSample extends OpenCvPipeline {
    ArrayList<double[]> frameList;
    public static double lowH = 30;
    public static double highH = 70;
    public static double lowS = 0;
    public static double highS = 255;
    public static double lowV = 0;
    public static double highV = 255;

    public static double threshold1 = 150;
    public static double threshold2 = 100;

    Telemetry telemetry;

    public DetectSample() {
        frameList = new ArrayList<>();
    }

    public void configure(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input){
        Mat mat = ComputerVision.convertColor(input, Imgproc.COLOR_RGB2HSV);
        Mat scaledThresh = ComputerVision.filterColor(mat, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV));
        Mat blurred = ComputerVision.blur(scaledThresh, new Size(5, 5));
        Pose2D position = ComputerVision.getPose(blurred);

        telemetry.addData("X", position.getX());
        telemetry.addData("Y", position.getY());
        telemetry.addData("Theta", position.getHeadingRadians());

        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        //RELEASE EVERYTHING

        input.release();
        mat.release();
        scaledThresh.release();
        blurred.copyTo(input);
        blurred.release();

        return input;
    }
}