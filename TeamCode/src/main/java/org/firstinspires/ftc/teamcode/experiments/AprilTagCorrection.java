package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.debug.ElevatedMecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "April Tag Correction")
public class AprilTagCorrection extends LinearOpMode {

    private boolean USE_WEBCAM = true;
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        ElevatedMecanumController weightedMecanumController = new ElevatedMecanumController(hardwareMap);
        weightedMecanumController.setDriveSpeed(0.3);
        double tolerance = 0.05;

        int emptyFrames = 0;

        initProcessors();

        telemetry.addLine("Processors Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            weightedMecanumController.driveWeights(gamepad1);

            AprilTagPoseFtc position = getAprilTagPosition(2, 4);
            if (position != null) {
                telemetry.addData("Yaw", position.yaw);
                telemetry.addData("Right", position.x * Constants.APRIL_TAG_POSITION_CORRECTION);
                telemetry.addData("Forward", position.y * Constants.APRIL_TAG_POSITION_CORRECTION);
                telemetry.update();

                double x = position.x * Constants.APRIL_TAG_POSITION_CORRECTION;
                double y = (position.y * Constants.APRIL_TAG_POSITION_CORRECTION) - 5;
                double r = position.yaw / 180;

                double distance = Math.sqrt((x * x) + (y * y) + (r * r));

                if (distance < tolerance) {
                    break;
                }

                weightedMecanumController.driveParamsFast(gamepad1, x, y, -r);
                emptyFrames = 0;
            } else {
                if (emptyFrames > 5) {
                    weightedMecanumController.deactivate();
                }
                emptyFrames += 1;
            }
        }

        visionPortal.close();
    }

    public AprilTagPoseFtc getAprilTagPosition(int... ids) {
        List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                for (int id: ids) {
                    if (detection.id == id) {
                        return detection.ftcPose;
                    }
                }
            }
        }

        return null;
    }

    public void initProcessors() {
        AprilTagLibrary.Builder myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Add tags in CenterStage
        myAprilTagLibraryBuilder.setAllowOverwrite(true);

        myAprilTagLibraryBuilder.addTag(1, "blue_alliance_left", 1+(18/32), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(2, "blue_alliance_center", 1+(18/32), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(3, "blue_alliance_right", 1+(18/32), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(4, "red_alliance_left", 1+(18/32), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(5, "red_alliance_center", 1+(18/32), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(6, "red_alliance_right", 1+(18/32), DistanceUnit.INCH);

        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor);

        visionPortal = builder.build();
    }
}
