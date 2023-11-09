package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Get Focal Length")
public class GetFocalLength extends LinearOpMode {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        tfod = new TfodProcessor.Builder()
                .setModelFileName("backdropDetection.tflite")
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(tfod);
        visionPortal = builder.build();

        telemetry.addLine("Initialized");
        telemetry.addLine("Please place the center of the camera rig EXACTLY 10 inches from the backdrop, and press play");
        telemetry.update();

        waitForStart();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        while(currentRecognitions.size() == 0) {
            currentRecognitions = tfod.getRecognitions();
        }
        Recognition currentRecognition = currentRecognitions.get(0);

        double focalLength = currentRecognition.getWidth() * 10 / Constants.BACKDROP_WIDTH;
        telemetry.addData("Focal Length Successfully Calculated", focalLength);
        telemetry.addLine("Please update the focal length in teamcode/debug/Constants...");
        telemetry.addLine("... not to be mistaken for teamcode/Constants");
        telemetry.update();

        sleep(30*1000);
    }
}
