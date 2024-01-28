package org.firstinspires.ftc.teamcode.autos.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PIDController;
import org.firstinspires.ftc.teamcode.shplib.vision.PIDFollower;
import org.firstinspires.ftc.teamcode.shplib.vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.concurrent.CopyOnWriteArrayList;

@Disabled
@Autonomous()
public class AdvancedPixelTracking extends LinearOpMode {
    private double euclidianDistance(double[] point1, double[] point2) {
        return Math.sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1]));
    }

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

        double[] lastObject = null;

        telemetry.addLine("initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            CopyOnWriteArrayList<double[]> objects = visionSubsystem.pixelDetectionPipeline.getObjects();

            if ((objects == null || objects.size() == 0)) {
                objects = visionSubsystem.pixelDetectionPipeline.getLastObjects();
            }

            double[] closestObject = null;
            double distance = -1;
            if (objects != null && objects.size() > 0) {
                for (double[] object : objects) {
                    if (object != null && object.length >= 2) {
                        if (lastObject == null) {
                            lastObject = object;
                            break;
                        }

                        if (distance == -1 || euclidianDistance(object, lastObject) < distance) {
                            distance = euclidianDistance(object, lastObject);
                            closestObject = object;
                        }
                    }
                }

                if (closestObject != null) {
                    pidFollower.update((closestObject[0] / 400) - 1,
                            1 - (closestObject[1] / 244),
                            0);

                    telemetry.addData("Object x", (closestObject[0] / 400) - 1);
                    telemetry.addData("Object y", 1 - (closestObject[1] / 244));
                    telemetry.addData("Object area", closestObject[2]);

                    lastObject = closestObject;
                } else {
                    cameraServo.setPower(0);
                }
            } else {
                mecanumController.driveParams(0, 0, 0);
                cameraServo.setPower(0);
            }

            telemetry.update();

            sleep(20);
        }
    }
}
