package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PIDController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.teamcode.shplib.vision.PIDFollower;
import org.firstinspires.ftc.teamcode.shplib.vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.concurrent.CopyOnWriteArrayList;

@Disabled
@TeleOp()
public class PixelTrackingRumbleFieldOriented extends LinearOpMode {
    private double euclidianDistance(double[] point1, double[] point2) {
        return Math.sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1]));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
        visionSubsystem.pixelDetectionPipeline.setPipelineMode(PixelDetectionPipeline.PipelineMode.PURPLE_ONLY);

        CRServo cameraServo = hardwareMap.get(CRServo.class, "cameraServo");

        PIDFollower pidFollower = new PIDFollower.PIDFollowerBuilder(
                null,
                cameraServo,
                new PIDController(0.30, 0, 0),
                new PIDController(0.18, 0, 0),
                new PIDController(0, 0, 0))
                .build();

        double[] lastObject = null;

        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.SINGLE_OVERRIDE)
                .setNaturalSpeed(0.6)
                .setOverrideOneSpeed(1)
                .build();

        MecanumController mecanumController = new MecanumController(hardwareMap, speedController);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_HIDDEN);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        CRServo air = hardwareMap.get(CRServo.class, "air");
        air.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor climber = hardwareMap.get(DcMotor.class, "climber");

        telemetry.addLine("initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Radians", mecanumController.getCalibratedIMUAngle());
            telemetry.update();

            mecanumController.fieldOrientedDrive(gamepad1);

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.OPEN_CLAW)) {
                claw.setPosition(Constants.CLAW_OPEN);
            } else if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLOSE_CLAW)) {
                claw.setPosition(Constants.CLAW_CLOSE);
            }

            if (gamepad1.b) {
                mecanumController.calibrateIMUAngleOffset();
            }

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLIMBER_POWER_UP)) {
                climber.setPower(1);
            } else if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLIMBER_POWER_STAY)) {
                climber.setPower(-0.5);
            } else if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLIMBER_POWER_DOWN)) {
                climber.setPower(-1);
            } else {
                climber.setPower(0);
            }

            air.setPower(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.AIR_POWER) ? 1: 0);

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
                    if (gamepad1.a) {
                        pidFollower.update((closestObject[0] / 400) - 1,
                                1 - (closestObject[1] / 244),
                                0);
                    } else {
                        cameraServo.setPower(0);
                    }

                    lastObject = closestObject;

                    gamepad1.rumble(closestObject[2], closestObject[2], 50);
                } else {
                    cameraServo.setPower(0);
                }


            } else {
                cameraServo.setPower(0);
            }
        }
    }
}
