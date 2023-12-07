package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.shplib.vision.BlueSpikeMarkerPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "Blue Alliance Left Side Autonomous")
public class BlueAllianceLeft extends LinearOpMode {
    TfodProcessor tfod;
    VisionPortal visionPortal;

    MecanumController mecanumController;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.3);

        Servo claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(Constants.CLAW_CLOSE);

        BlueSpikeMarkerPipeline visionPipeline = new BlueSpikeMarkerPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(visionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(800, 440, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        mecanumController.calibrateIMUAngleOffset();

        mecanumController.moveInches(5, 5, 5, 5, true);

        int spikeLocation = visionPipeline.getLocation();
        telemetry.addData("spike location", spikeLocation);
        telemetry.update();

        if (spikeLocation == 1) {

            mecanumController.moveInches(26.75, 26.75, 26.75, 26.75, true);
            mecanumController.calibrateIMUAngleOffset();
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(1000);
            mecanumController.moveInches(-3, -3, -3, -3, true);

        } else if (spikeLocation == 2) {

            mecanumController.moveInches(21.75, 21.75, 21.75, 21.75, true);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(1000);
            mecanumController.moveInches(-3, -3, -3, -3, true);

        } else if (spikeLocation == 3) {

            mecanumController.moveInches(26.75, 26.75, 26.75, 26.75, true);
            mecanumController.calibrateIMUAngleOffset();
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(1000);
            mecanumController.moveInches(-3, -3, -3, -3, true);

        }
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName("blue.tflite")
                .setModelLabels(new String[]{"blue_spike_marker"})
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(tfod);

        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private int getSpikeLocation() {
        mecanumController.rotateToRadian(Math.toRadians(30), Math.toRadians(1));

        // TODO: tune this val if not detecting
        sleep(0);

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        mecanumController.rotateToRadian(Math.toRadians(0), Math.toRadians(1));

        if (currentRecognitions.size() >= 1) {
            return 1;
        }

        // TODO: tune this val if not detecting
        sleep(0);

        currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() >= 1) {
            return 2;
        }

        return 3;
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
}