package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.debug.LinearSlide;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.Speed;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
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

@TeleOp(name = "CenterStage Field Oriented")
public class CenterstageFieldOriented extends LinearOpMode {

    private boolean USE_WEBCAM = true;
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, Speed.SINGLE_OVERRIDE);
        mecanumController.calibrateIMUAngleOffset();
        mecanumController.setDriveSpeed(1);

        LinearSlide lift = new LinearSlide(hardwareMap, "lift", true);

        //SampleMecanumDrive roadrunnerCorrection = new SampleMecanumDrive(hardwareMap);

        //initProcessors();

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

            lift.drive(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LIFT_POWER));
        }
    }
}
