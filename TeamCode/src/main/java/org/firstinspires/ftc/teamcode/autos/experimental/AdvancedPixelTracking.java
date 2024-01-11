package org.firstinspires.ftc.teamcode.autos.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PIDController;
import org.firstinspires.ftc.teamcode.shplib.vision.PIDFollower;
import org.firstinspires.ftc.teamcode.shplib.vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.concurrent.CopyOnWriteArrayList;

@Autonomous()
public class AdvancedPixelTracking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
        visionSubsystem.pixelDetectionPipeline.setPipelineMode(PixelDetectionPipeline.PipelineMode.PURPLE_ONLY);

        MecanumController mecanumController = new MecanumController(hardwareMap);
        CRServo cameraServo = hardwareMap.get(CRServo.class, "cameraServo");

        PIDFollower pidFollower = new PIDFollower.PIDFollowerBuilder(
                mecanumController,
                cameraServo,
                new PIDController(0.30, 0, 0),
                new PIDController(0.18, 0, 0),
                new PIDController(0, 0, 0))
                .build();

        telemetry.addLine("initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            CopyOnWriteArrayList<double[]> objects = visionSubsystem.pixelDetectionPipeline.getObjects();

            if ((objects == null || objects.size() == 0)) {
                objects = visionSubsystem.pixelDetectionPipeline.getLastObjects();
            }

            if (objects != null && objects.size() > 0) {
                for (double[] object : objects) {
                    if (object != null && object.length >= 2) {
                        pidFollower.update((object[0] / 400) - 1,
                                1 - (object[1] / 244),
                                0);

                        telemetry.addData("Object x", (object[0] / 400) - 1);
                        telemetry.addData("Object y", 1 - (object[1] / 244));
                        telemetry.addData("Object area", object[2]);
                    }
                }
            } else {
                mecanumController.driveParams(0, 0, 0);
            }

            telemetry.update();

            sleep(20);
        }
    }
}
