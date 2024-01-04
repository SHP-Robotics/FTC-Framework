package org.firstinspires.ftc.teamcode.autos.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PIDController;
import org.firstinspires.ftc.teamcode.shplib.vision.PIDFollower;
import org.firstinspires.ftc.teamcode.shplib.vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.concurrent.CopyOnWriteArrayList;

@Autonomous()
public class AdvancedPixelTracking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
        visionSubsystem.pixelDetectionPipeline.setTrackingCenters(true);
        visionSubsystem.pixelDetectionPipeline.setPipelineMode(PixelDetectionPipeline.PipelineMode.PURPLE_ONLY);

        MecanumController mecanumController = new MecanumController(hardwareMap);
        PIDFollower pidFollower = new PIDFollower.PIDFollowerBuilder(mecanumController,
                new PIDController(0.4, 0, 0.2),
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            CopyOnWriteArrayList<double[]> objects = visionSubsystem.pixelDetectionPipeline.getObjects();

            if ((objects == null || objects.size() == 0)) {
                objects = visionSubsystem.pixelDetectionPipeline.getLastObjects();
            }

            if (objects != null && objects.size() > 0) {
                for (double[] object : objects) {
                    if (object != null && object.length >= 2) {
                        pidFollower.update((((double)object[0])/400)-1, 0, 0);

                        telemetry.addData("Object x", object[0]);
                        telemetry.addData("Object y", object[1]);
                        telemetry.addData("Object area", object[2]);
                    } else {
                        mecanumController.driveParams(0, 0, 0);
                    }
                }
            }

            telemetry.update();

            sleep(20);
        }
    }
}
