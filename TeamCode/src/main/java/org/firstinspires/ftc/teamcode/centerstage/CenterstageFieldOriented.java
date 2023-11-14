package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.Speed;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "CenterStage Field Oriented")
public class CenterstageFieldOriented extends LinearOpMode {

    private boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, Speed.SINGLE_OVERRIDE);
        mecanumController.calibrateIMUAngleOffset();
        mecanumController.setDriveSpeed(1);

        //Synchronous climber = new Synchronous(hardwareMap, "leftClimber", "rightClimber");
        //climber.setMotorDirection(Side.LEFT, DcMotorSimple.Direction.REVERSE);

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.fieldOrientedDrive(gamepad1);
            telemetry.addData("radians clockwise", mecanumController.getCalibratedIMUAngle());
            telemetry.update();

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.RESET_IMU)) {
                // I wonder if this will reset the IMU if the Yaw is off after a collision
                // Further testing required
                // mecanumController.initIMU(hardwareMap);
                mecanumController.calibrateIMUAngleOffset();
            }

            if (gamepad1.b) {
                AprilTagPoseFtc position = getCenterPosition();
                if (position != null) {
                    mecanumController.moveToPosition(position.x, position.y - 20, true);
                    mecanumController.fakeReset();
                }
            }

            //climber.setPowerSynchronous(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLIMBER_POWER));
        }

        visionPortal.close();
    }

    public AprilTagPoseFtc getCenterPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 2) {
                return detection.ftcPose;
            }
        }

        return null;
    }
}
