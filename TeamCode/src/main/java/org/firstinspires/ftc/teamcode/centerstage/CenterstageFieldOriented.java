package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.OneMotorSystem;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "CenterStage Field Oriented")
public class CenterstageFieldOriented extends LinearOpMode {

    private boolean USE_WEBCAM = true;
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.GEAR_SHIFT)
                .setNaturalSpeed(1)
                .setGearSpacing(0.1)
                .build();

        MecanumController mecanumController = new MecanumController(hardwareMap, speedController);

        Synchronous climber = new Synchronous(hardwareMap, "leftClimber", "rightClimber");
        climber.setMotorDirection(Side.LEFT, DcMotorSimple.Direction.REVERSE);

        OneMotorSystem lift = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "lift")
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setUseBrakes(true)
                .setUseEncoders(false)
                .setStaticPower(0.3)
                .build();

        OneMotorSystem intake = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "intake")
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setUseEncoders(false)
                .build();

        //SampleMecanumDrive roadrunnerCorrection = new SampleMecanumDrive(hardwareMap);

        //initProcessors();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("radians clockwise", mecanumController.getCalibratedIMUAngle());
            telemetry.update();

            if (gamepad1.x) {

                mecanumController.deactivate();
                lift.drive(0);
                intake.drive(0);

                climber.leftMotor.setPower(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LEFT_CLIMBER_POWER));
                climber.rightMotor.setPower(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.RIGHT_CLIMBER_POWER));

            } else {

                mecanumController.fieldOrientedDrive(gamepad1);
                lift.drive(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LIFT_POWER));
                intake.drive(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.INTAKE_POWER_FORWARDS)
                        - DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.INTAKE_POWER_BACKWARDS));

                climber.setPowerSynchronous(0);

                if (gamepad1.b) {
                    mecanumController.calibrateIMUAngleOffset();
                }

            }

            /*
            if (gamepad1.x) {
                AprilTagPoseFtc position = getAprilTagPosition(2, 4);
                if (position != null) {
                    //telemetry.addData("yaw", position.yaw);
                    //telemetry.addData("Forward", position.y/25.4);
                    //telemetry.addData("Right", position.z/25.4);
                    //telemetry.update();

                    Pose2d startPose = new Pose2d(0, 0, 0);

                    TrajectorySequence trajSeq = roadrunnerCorrection.trajectorySequenceBuilder(startPose)
                            .lineTo(new Vector2d(position.y+0.5, -position.x))
                            .build();

                    roadrunnerCorrection.setPoseEstimate(startPose);
                    roadrunnerCorrection.turn(Math.toRadians(-position.yaw));
                    roadrunnerCorrection.followTrajectorySequence(trajSeq);
                }
            }
            */
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

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(myAprilTagProcessor);

        visionPortal = builder.build();
    }
}
