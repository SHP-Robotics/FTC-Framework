package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectSample extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input){
//        Mat mat = new Mat();

        //From RGB to HSV to for better tuning
        //EasyOpenCV hue is 0-180
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Lower and upper bounds for the color to detect
//        Scalar lowHSV = new Scalar(210.0/2, 50, 70);
//        Scalar highHSV = new Scalar(250.0/2, 255, 255);

//        Mat detected = new Mat();
//        Core.inRange(mat, lowHSV, highHSV, detected); //ONLY returns the pixels in the HSV range

//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(detected, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//        int maxHeight = 0;
//        for (MatOfPoint contour: contours) {
//            int height = contour.rows();
//            if (height > maxHeight) {
//                maxHeight = height;
//            }
//        }
//        isReadable = false;
//        maxHeightReadable = maxHeight;
//        isReadable = true;

//        Mat left = detected.submat(LEFT_ROI);
//        Mat right = detected.submat(RIGHT_ROI);

//        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 225;
//        rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 225;
//        totalValue = Core.sumElems(detected).val[0] / (detected.rows() * detected.cols() * 255);

        //        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

//        Scalar colorExists = new Scalar (0, 255, 0);
//        Scalar colorInexistant = new Scalar (255, 0, 0);

//        Imgproc.rectangle(detected, LEFT_ROI, leftValue>rightValue? colorInexistant:colorExists, 3);
//        Imgproc.rectangle(detected, RIGHT_ROI, leftValue<rightValue? colorInexistant:colorExists, 3);

        //RELEASE EVERYTHING
//        input.release();
//        mat.release();
//        detected.copyTo(input);
//        detected.release();
//        left.release();
//        right.release();

        return input;
    }
}