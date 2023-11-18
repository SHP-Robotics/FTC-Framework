package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.Speed;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {

    private boolean USE_WEBCAM = true;
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        initProcessors();

        telemetry.addLine("Processors Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            AprilTagPoseFtc position = getAprilTagPosition(2, 4);
            if (position != null) {
                telemetry.addData("Yaw", position.yaw);
                telemetry.addData("Right", position.x * Constants.APRIL_TAG_POSITION_CORRECTION);
                telemetry.addData("Forward", position.y * Constants.APRIL_TAG_POSITION_CORRECTION);
                telemetry.update();
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
