package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.vision.ComputerVision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
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
        Mat blurred = ComputerVision.blur(scaledThresh, new Size(1, 1));

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blurred, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (MatOfPoint contour: contours) {
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(points);
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            if (rotatedRect.size.height < 100 || rotatedRect.size.width < 100)
                continue;
            telemetry.addLine("h" + rotatedRect.size.height + " w" + rotatedRect.size.width);
            drawRotatedRect(rotatedRect, input, new Scalar(255, 0, 0), 3);
            contour2f.release();
        }

        Pose2D position = ComputerVision.getPose(blurred);

        telemetry.addData("X", position.getX());
        telemetry.addData("Y", position.getY());
        telemetry.addData("Theta", position.getHeadingRadians());
        telemetry.update();

        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        //RELEASE EVERYTHING

//        input.release();
        mat.release();
        scaledThresh.release();
        blurred.release();

        return input;
    }

    static void drawRotatedRect(RotatedRect rect, Mat mat, Scalar color) {
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(mat, points[i], points[(i + 1) % 4], color, 2);
        }
    }

    static void drawRotatedRect(RotatedRect rect, Mat mat, Scalar color, int thickness) {
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(mat, points[i], points[(i + 1) % 4], color, thickness);
        }
    }
}