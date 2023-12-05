package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineObjectDetection extends OpenCvPipeline {
    public double leftValue, rightValue;
    Mat mat = new Mat();
    Mat mat1 = new Mat();
    Mat mat2 = new Mat();
    //sub matrices to divide image
    static final Rect LEFT_ROI = new Rect(
            new Point(1, 1),
            new Point(399, 447)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point(400, 1),
            new Point(799, 447)
    );
    static final double threshold = 0.2;
    public CVPipelineObjectDetection(){
        leftValue = 0;
        rightValue = 0;
    }
    @Override
    public Mat processFrame(Mat input){
        //rgb to hsv
        Imgproc.cvtColor(input, mat1, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, mat2, Imgproc.COLOR_RGB2HSV);
       //blue value
        //Scalar lowHSV= new Scalar(95, 100, 100);
        //Scalar highHSV= new Scalar(112.5, 255, 255);
        //end value - 100(darkest) and 255(lightest) - value
        //start value -100(lightest saturation) and 255(darkest)
        //red value
        Scalar lowHSV1= new Scalar(0, 100, 100);
        Scalar highHSV1= new Scalar(10, 255, 255);
        Scalar lowHSV2= new Scalar(170, 100, 100);
        Scalar highHSV2= new Scalar(180, 255, 255);
        //yellow
//        Scalar lowHSV = new Scalar(23, 50, 70);
//        Scalar highHSV = new Scalar(32, 255, 255);
        //threshold - erase everything expect something that is yellow
        //white/black image with white = in range and black = out of range
        Core.inRange(mat1, lowHSV1, highHSV1, mat1);
        Core.inRange(mat2, lowHSV2, highHSV2, mat2);
        //extract regions of interest
        Mat left1 = mat1.submat(LEFT_ROI);
        Mat right1 = mat1.submat(RIGHT_ROI);
        Mat left2 = mat2.submat(LEFT_ROI);
        Mat right2 = mat2.submat(RIGHT_ROI);
        //check to see what percentage of the matrix is white
//        leftSum = Core.sumElems(left).val[0];
//        rightSum = Core.sumElems(right).val[0];
        leftValue = (Core.sumElems(left1).val[0]+Core.sumElems(left2).val[0])/ LEFT_ROI.area() / 255;
        rightValue = (Core.sumElems(right1).val[0]+Core.sumElems(right2).val[0])/ RIGHT_ROI.area() / 255;
        //release submatrices
//        left.release();
//        right.release();
        //telemetry display calculation vals
//        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
//        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
//        telemetry.addData("Left percentage", Math.round(leftValue*100) + "%");
//        telemetry.addData("Right percentage", Math.round(rightValue*100)+"%");
        //identify whether the object is there - higher than threshold
        boolean objectLeft = leftValue >= threshold;
        boolean objectRight = rightValue >= threshold;
        //convert grayscale to rgb
        Imgproc.cvtColor(mat1, mat1, Imgproc.COLOR_GRAY2RGB);
        //make rects on screen
        Scalar objHere = new Scalar(0, 255, 0);
        Scalar objNot = new Scalar(255, 0, 0);
        Imgproc.rectangle(mat1, LEFT_ROI, objectLeft==true ? objHere:objNot);
        Imgproc.rectangle(mat1, RIGHT_ROI, objectRight==true ? objHere:objNot);
        return mat1;
    }
    //return place
//    public loc getLocation(){
//        return location;
//    }
    public double getRightVal(){return rightValue;}
    public double getLeftVal(){return leftValue;}
}
