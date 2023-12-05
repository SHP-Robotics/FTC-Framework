package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineObjectTemplate extends OpenCvPipeline {
    //    Telemetry telemetry;
    public double leftValue;
    public double rightValue;
    Mat mat = new Mat();
    //    public enum loc{
//        RIGHT,
//        LEFT,
//        NONE
//    };
//    loc location;
    //sub matrices to divide image
    //we can draw rectangles on the screen to find the perfect fit
    //edit as necessary
    static final Rect LEFT_ROI = new Rect(
            new Point(1, 1),
            new Point(399, 447)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point(400, 1),
            new Point(799, 447)
    );
    //try this if other looks funky
//    static final Rect LEFT_ROI = new Rect(1, 1, 399, 447);
//    static final Rect RIGHT_ROI = new Rect(400, 1, 399, 447);
    //threshold(lowest possible) percentage of that color
    static double threshold = 0.2;

    //check if we really need this

    @Override
    public Mat processFrame(Mat input){
        //rgb to hsv
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //scalar range represented below
        //x values = hue
        //y values = saturation
        //z values = range of Value
        //try to keep y and z the same
        //change hue values - right now they are set to yellow
        //dark blue - 100 to 140
        //may need to change these
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);
        //threshold - erase everything expect something that is yellow
        //white/black image with white = in range and black = out of range
        Core.inRange(mat, lowHSV, highHSV, mat);
        //extract regions of interest
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        //check to see what percentage of the matrix is white
//        leftSum = Core.sumElems(left).val[0];
//        rightSum = Core.sumElems(right).val[0];
        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        //release submatrices
        left.release();
        right.release();
        //telemetry display calculation vals
//        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
//        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
//        telemetry.addData("Left percentage", Math.round(leftValue*100) + "%");
//        telemetry.addData("Right percentage", Math.round(rightValue*100)+"%");
        //identify whether the object is there - higher than threshold
        boolean objectLeft = leftValue >= threshold;
        boolean objectRight = rightValue >= threshold;
        //convert grayscale to rgb
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        //make rects on screen
        Scalar objHere = new Scalar(0, 255, 0);
        Scalar objNot = new Scalar(255, 0, 0);
        Imgproc.rectangle(mat, LEFT_ROI, objectLeft==true ? objHere:objNot);
        Imgproc.rectangle(mat, RIGHT_ROI, objectRight==true ? objHere:objNot);
        return mat;
    }
    //return place
//    public loc getLocation(){
//        return location;
//    }
    public double getRightVal(){return rightValue;}
    public double getLeftVal(){return leftValue;}
}
