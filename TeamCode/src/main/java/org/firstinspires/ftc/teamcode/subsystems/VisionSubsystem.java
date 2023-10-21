package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class VisionSubsystem extends Subsystem {
    private final OpenCvCamera camera;
    private final AprilTagDetectionPipeline pipeline;
    private ArrayList<AprilTagDetection> tags;

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

    public VisionSubsystem(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(Constants.Vision.kTagsizeMeters, fx, fy, cx, cy);
        tags = new ArrayList<>();

        camera.setPipeline(pipeline);
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

    public boolean detectedTags() {
        return !tags.isEmpty();
    }

    public ArrayList<AprilTagDetection> getTags() {
        return tags;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        if (state == State.DISABLED) return;
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if (detections != null) {
            // If we don't see any tags
            if (detections.size() == 0) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    pipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    pipeline.setDecimation(DECIMATION_HIGH);
                }

//                for (AprilTagDetection detection : detections) {
//                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//                }

                tags = detections;
            }
        }
    }
}
