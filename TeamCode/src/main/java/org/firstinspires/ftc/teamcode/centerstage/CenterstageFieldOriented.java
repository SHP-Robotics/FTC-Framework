package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.OneMotorSystem;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
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
    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();

        while (opModeIsActive()) {
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
        }
    }
}
