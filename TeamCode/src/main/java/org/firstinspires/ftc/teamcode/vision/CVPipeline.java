package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipeline extends OpenCvPipeline{
    //    Telemetry telemetry;
    Mat YCbCr = new Mat();
    //screen is divided into two boxes
    Mat leftCrop;
    Mat rightCrop;

    double leftavgfin;
    double rightavgfin;
    Mat outPut = new Mat();
    //testing for red?
    Scalar rectColor = new Scalar (255.0, 0.0, 0.0);

//    public CVPipeline(Telemetry t) {telemetry = t;}
    @Override
    public Mat processFrame(Mat input){
        //camera width is 800
        //camera height is 448
        Imgproc.cvtColor(input,YCbCr, Imgproc.COLOR_RGB2YCrCb);
//        telemetry.addLine("pipeline running");

        //left and right rectangle test
        Rect leftRect = new Rect(1, 1, 399, 447);
        Rect rightRect = new Rect(400, 1, 399, 447);

        //get rid of later - allows us to see bounding boxes
        input.copyTo(outPut);
        Imgproc.rectangle(outPut, leftRect, rectColor, 2);
        Imgproc.rectangle(outPut, rightRect, rectColor, 2);

        // - these crops are submaps
        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);

        //remove everything but red - extracting channel 2 - red
        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        //average of red in channel
        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);

        //find out red in each submap
        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];

//        if(leftavgfin > rightavgfin){
//            telemetry.addLine("left");
//        }
//        else{
//            telemetry.addLine("right");
//        }

        //returns the current frame we just made
        return(outPut);
    }

    public boolean isLeft(){
        return leftavgfin > rightavgfin;
    }
}
