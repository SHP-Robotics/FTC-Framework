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

    public void dropPurplePixel() {
        fourBarLinkage.setPosition(Constants.DUAL_PIXEL_STORAGE_POSITION);
        claw.setPosition(Constants.CLAW_OPEN);
        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.SHALLOW_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0.3);

        claw.setPosition(Constants.CLAW_CLOSE);
        lift.setPosition((int) (Constants.LOW_BONUS_HEIGHT), true);
        lift.setLiftPower(0.3);
        fourBarLinkage.setPosition(Constants.BEHIND_DUAL_PIXEL_STORAGE_POSITION);

        claw.setPosition(Constants.CLAW_OPEN);

        fourBarLinkage.setPosition(Constants.DUAL_PIXEL_STORAGE_POSITION);
        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.DEEP_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0);
    }

    public void dropYellowPixel() {
        fourBarLinkage.setPosition(Constants.DUAL_PIXEL_STORAGE_POSITION);
        claw.setPosition(Constants.CLAW_OPEN);
        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.DEEP_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0.3);

        claw.setPosition(Constants.CLAW_CLOSE);
        lift.setPosition((int) (Constants.LOW_BONUS_HEIGHT), true);
        lift.setLiftPower(0.3);
        fourBarLinkage.setPosition(Constants.OUTTAKE_POSITION);

        claw.setPosition(Constants.CLAW_OPEN);

        fourBarLinkage.setPosition(Constants.DUAL_PIXEL_STORAGE_POSITION);
        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.DEEP_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController primitiveMecanumDrive = new MecanumController(hardwareMap);
        SampleMecanumDrive roadrunnerMecanumDrive = new SampleMecanumDrive(hardwareMap);

        lift = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "lift")
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .build();

        fourBarLinkage = hardwareMap.get(Servo.class, "fourBarLinkage");
        claw = hardwareMap.get(Servo.class, "claw");

        ColorSensor colorSensorForward = hardwareMap.get(ColorSensor.class, "colorSensorForward");

        TrajectorySequence initialization = roadrunnerMecanumDrive.trajectorySequenceBuilder(roadrunnerMecanumDrive.getPoseEstimate())
                // align with center spike mark
                .forward(24)
                .addDisplacementMarker(() -> {
                    if (colorSensorForward.blue() > 1200) {
                        spikeLocation = 2;
                        dropPurplePixel();
                    }
                })
                .turn(Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    if (colorSensorForward.blue() > 1200) {
                        spikeLocation = 1;
                        dropPurplePixel();
                    }
                })
                .forward(48)
                .build();

        telemetry.addLine("Trajectories built");
        telemetry.update();

        initProcessors();

        telemetry.addLine("Trajectories built");
        telemetry.addLine("Processors initialized");
        telemetry.addLine("Ready to start!");
        telemetry.update();


        waitForStart();

        roadrunnerMecanumDrive.followTrajectorySequence(initialization);

        int count = 0;
        double avgX = 0;
        double avgY = 0;

        ElapsedTime oneSecondTimer = new ElapsedTime();
        oneSecondTimer.reset();

        while (oneSecondTimer.seconds() < 1) {
            AprilTagPoseFtc position = getAprilTagPosition(2);
            if (position != null) {
                telemetry.addData("yaw", position.yaw);
                telemetry.addData("Forward", position.y/25.4);
                telemetry.addData("Right", position.z/25.4);
                telemetry.update();
                count += 1;
            }
            sleep(5);
        }

        if (spikeLocation == 1) {
            avgX -= 3;
        } else if (spikeLocation == 3) {
            avgX += 3;
        }

        if (count != 0) {
            avgX /= count;
            avgY /= count;
        }

        TrajectorySequence aprilTagAlignment = roadrunnerMecanumDrive.trajectorySequenceBuilder(new Pose2d(0, 0,roadrunnerMecanumDrive.getPoseEstimate().getHeading()))
            .lineTo(new Vector2d(avgY, avgX))
            /*
            .addDisplacementMarker(() -> {
                // Currently ~1inch right of April Tag to be safe
                // Finish alignment Y axis (left-right) with color sensor
                while (colorSensorForward.blue() < 1200) {
                    primitiveMecanumDrive.leftFront.setPower(-0.3);
                    primitiveMecanumDrive.rightFront.setPower(0.3);
                    primitiveMecanumDrive.leftRear.setPower(0.3);
                    primitiveMecanumDrive.rightRear.setPower(-0.3);
                }

                while (colorSensorForward.blue() > 1200) {
                    primitiveMecanumDrive.leftFront.setPower(0.3);
                    primitiveMecanumDrive.rightFront.setPower(-0.3);
                    primitiveMecanumDrive.leftRear.setPower(-0.3);
                    primitiveMecanumDrive.rightRear.setPower(0.3);
                }

                dropYellowPixel();
            })
             */
            .build();

        roadrunnerMecanumDrive.followTrajectorySequence(aprilTagAlignment);
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
