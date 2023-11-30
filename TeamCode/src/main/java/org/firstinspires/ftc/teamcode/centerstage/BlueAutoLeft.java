package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.OneMotorSystem;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Autonomous(name = "Blue Alliance Left Autonomous")
public class BlueAutoLeft extends LinearOpMode {
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    private VisionPortal visionPortal;

    int spikeLocation = 3;

    OneMotorSystem lift;
    Servo fourBarLinkage;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController primitiveMecanumDrive = new MecanumController(hardwareMap);

        lift = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "lift")
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .build();

        claw = hardwareMap.get(Servo.class, "claw");

        ColorSensor colorSensorForward = hardwareMap.get(ColorSensor.class, "colorSensorForward");

        CenterstageMacros centerstageMacros = new CenterstageMacros.CenterstageRobotBuilder(primitiveMecanumDrive)
                .setLift(lift)
                .setClaw(claw)
                .setAprilTagProcessor(myAprilTagProcessor)
                .setColorSensor(colorSensorForward)
                .build();

        initProcessors();

        telemetry.addLine("Processors initialized");
        telemetry.addLine("Ready to start!");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            AprilTagPoseFtc position = centerstageMacros.getAprilTagPosition(2);
            if (position != null) {
                telemetry.addData("yaw", position.yaw);
                telemetry.addData("X", position.x);
                telemetry.addData("Y", position.y);
                telemetry.addData("Z", position.z);
                telemetry.update();
            }
            sleep(5);
        }
    }

    public void initProcessors() {
        AprilTagLibrary.Builder myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Add tags in CenterStage
        myAprilTagLibraryBuilder.setAllowOverwrite(true);

        myAprilTagLibraryBuilder.addTag(1, "blue_alliance_left", 1+(13/16), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(2, "blue_alliance_center", 1+(13/16), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(3, "blue_alliance_right", 1+(13/16), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(4, "red_alliance_left", 1+(13/16), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(5, "red_alliance_center", 1+(13/16), DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(6, "red_alliance_right", 1+(13/16), DistanceUnit.INCH);

        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.addProcessor(myAprilTagProcessor);

        visionPortal = builder.build();
    }
}
