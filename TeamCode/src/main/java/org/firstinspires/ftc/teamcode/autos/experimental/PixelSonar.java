package org.firstinspires.ftc.teamcode.autos.experimental;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.shplib.vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous()
public class PixelSonar extends LinearOpMode {
    MecanumController mecanumController;
    VisionSubsystem visionSubsystem;
    Servo claw;

    public void pixelSonar() {
        boolean movedYet = false;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("total value", visionSubsystem.pixelDetectionPipeline.valueTotal);
            telemetry.update();

            PixelDetectionPipeline.PixelMassLocation pixelMassLocation = visionSubsystem.getPixelMassLocation();
            if (pixelMassLocation == PixelDetectionPipeline.PixelMassLocation.NONE) {
                if (movedYet) {
                    mecanumController.driveParams(0, 0, 0);
                    break;
                } else {
                    mecanumController.driveParams(0, 0, 0.15);
                }
            }

            if (pixelMassLocation == PixelDetectionPipeline.PixelMassLocation.LEFT) {
                mecanumController.driveParams(0, 0, -0.15);
                movedYet = true;
            }

            if (pixelMassLocation == PixelDetectionPipeline.PixelMassLocation.CENTER) {
                mecanumController.driveParams(0, 0.2, 0);
                movedYet = true;
            }

            if (pixelMassLocation == PixelDetectionPipeline.PixelMassLocation.RIGHT) {
                mecanumController.driveParams(0, 0, 0.15);
                movedYet = true;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumController = new MecanumController(hardwareMap);
        visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        pixelSonar();
        claw.setPosition(Constants.CLAW_CLOSE);
        sleep(200);
    }
}
