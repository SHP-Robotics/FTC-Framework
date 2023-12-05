package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.vision.CVPipelineObjectDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CameraOn extends Subsystem {
    private final OpenCvCamera camera;
    static double threshold = 0.2;
    //    CVPipeline pipeline = new CVPipeline();
    CVPipelineObjectDetection detector = new CVPipelineObjectDetection();

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;

    private int numFramesWithoutDetection = 0;

    private final float DECIMATION_HIGH = 3;
    private final float DECIMATION_LOW = 2;
    private final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    private final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public enum State {
        ENABLED, DISABLED
    }

    private State state;

    public CameraOn(HardwareMap hardwareMap) {
        //live streams camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //pause or resume webstream
        //webcam.resumeViewport();
        //camera.pauseViewport();

        //creating a camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //setPipeline/create cit
        //basic pipeline tells us where the object is on the screen
//        camera.setPipeline(pipeline);
        //another possible pipeline
        //CVPipelineObjectDetection detector = new CVPipelineObjectDetection(telemetry);
        camera.setPipeline(detector);
        //opens camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        setState(State.DISABLED);
    }

    public void setState(State state) {
        this.state = state;
    }

//    public boolean loc(){
//        return pipeline.isLeft();
//    }

    public void showRes(Telemetry telemetry){
        if(detector.getLeftVal()>=threshold){
            telemetry.addData("location:", "left");
            telemetry.addData("left pixel sum:", detector.getLeftVal());
            telemetry.addData("right pixel sum:", detector.getRightVal());
        }
        else if(detector.getRightVal()>= threshold){
            telemetry.addData("location:", "right");
            telemetry.addData("left pixel sum:", detector.getLeftVal());
            telemetry.addData("right pixel sum:", detector.getRightVal());
        }
        else if(detector.getRightVal()>= threshold && detector.getLeftVal()>=threshold){
            telemetry.addData("location:", "all");
            telemetry.addData("left pixel sum:", detector.getLeftVal());
            telemetry.addData("right pixel sum:", detector.getRightVal());
        }
        else{
            telemetry.addData("location:", "none");
            telemetry.addData("left pixel sum:", detector.getLeftVal());
            telemetry.addData("right pixel sum:", detector.getRightVal());
        }
    }
    @Override
    public void periodic(Telemetry telemetry) {

        if (state == State.DISABLED) return;
        //only for second pipeline
        //check if we need to update camera or if it just updates automatically
//        switch(detector.getLocation()){
//            case LEFT{
//                break;
//            }
//            case RIGHT{
//                break;
//            }
//            case NONE{
//                break;
//            }
//        }
//        why doesn't this automatically display?
//        if(detector.getLocation()==CVPipelineObjectDetection.loc.LEFT){
//            telemetry.addLine("left");
//            telemetry.addData("left pixel sum:", detector.getLeftSum());
//            telemetry.addData("right pixel sum:", detector.getRightSum());
//        }
//        else if(detector.getLocation()==CVPipelineObjectDetection.loc.RIGHT){
//            telemetry.addLine("right");
//            telemetry.addData("left pixel sum:", detector.getLeftSum());
//            telemetry.addData("right pixel sum:", detector.getRightSum());
//        }
//        else{
//            telemetry.addLine("none");
//            telemetry.addData("left pixel sum:", detector.getLeftSum());
//            telemetry.addData("right pixel sum:", detector.getRightSum());
//        }

    }
}