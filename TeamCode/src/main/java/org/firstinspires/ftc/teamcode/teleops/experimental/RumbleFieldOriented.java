package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.teamcode.shplib.vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.concurrent.CopyOnWriteArrayList;

@TeleOp()
public class RumbleFieldOriented extends LinearOpMode {
    private double euclideanDistance(double[] point1, double[] point2) {
        return Math.sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1]));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
        visionSubsystem.pixelDetectionPipeline.setPipelineMode(PixelDetectionPipeline.PipelineMode.ALL_PIXELS);
        boolean rumbleEnabled;

        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(Constants.CameraMode.FACING_CLAW.getPosition());

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

            rumbleEnabled = gamepad1.left_trigger > 0.1;

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

                        if (distance == -1 || euclideanDistance(object, lastObject) < distance) {
                            distance = euclideanDistance(object, lastObject);
                            closestObject = object;
                        }
                    }
                }

                if (closestObject != null) {
                    lastObject = closestObject;

                    if (rumbleEnabled) {
                        gamepad1.rumble(closestObject[2], closestObject[2], 50);
                    }
                }
            }
        }
    }
}
