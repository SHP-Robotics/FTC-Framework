package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Get Distance")
public class GetDistance extends LinearOpMode {
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

        List<Recognition> currentRecognitions;
        while(opModeIsActive()) {
            currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions) {
                telemetry.addData("Distance", Constants.FOCAL_LENGTH * Constants.BACKDROP_WIDTH / recognition.getWidth());
            }
            telemetry.update();
        }
    }
}
