package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.teamcode.shplib.vision.PixelDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp()
public class RumbleFieldOriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
        visionSubsystem.pixelDetectionPipeline.setPipelineMode(PixelDetectionPipeline.PipelineMode.PURPLE_ONLY);
        boolean rumbleEnabled = true;
        boolean detected = false;
        double rumbleMultiplier = 1;
        final double THRESHOLD = 0.015;
        double maxDetectionTime = 5;
        double minTimeBetweenDetections = 2;

        ElapsedTime timer = new ElapsedTime();

        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setDirection(Servo.Direction.REVERSE);
        cameraServo.setPosition(Constants.CameraMode.FACING_CLAW.getPosition());

        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.SINGLE_OVERRIDE)
                .setNaturalSpeed(0.6)
                .setOverrideOneSpeed(1)
                .build();

        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setSpeedController(speedController);
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

            // Rumble
            double value = visionSubsystem.pixelDetectionPipeline.getPixelMass();
            if (value > THRESHOLD) {
                if (rumbleEnabled) {
                    gamepad1.rumble(Math.min(value * rumbleMultiplier, 1), Math.min(value * rumbleMultiplier, 1), 50);
                }

                if (!detected && !rumbleEnabled) {
                    timer.reset();
                }
                if (timer.seconds() > maxDetectionTime) {
                    rumbleEnabled = false;
                }

                detected = true;
            } else {
                if (detected) {
                    timer.reset();
                }
                if (timer.seconds() > minTimeBetweenDetections) {
                    rumbleEnabled = true;
                }
                gamepad1.rumble(0, 0, 50);
                detected = false;
            }
        }
    }
}
