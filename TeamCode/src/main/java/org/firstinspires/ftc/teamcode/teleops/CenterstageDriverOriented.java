package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.OneMotorSystem;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.io.File;

@TeleOp(name = "CenterStage Driver Oriented")
public class CenterstageDriverOriented extends LinearOpMode {
    MecanumController mecanumController;
    VisionSubsystem visionSubsystem;

    final double minimumPixelMass = 0.2;

    public void pixelSonar() {
        while (gamepad1.y && opModeIsActive() && !isStopRequested()) {
            if (visionSubsystem.getPixelMass() > minimumPixelMass) {
                mecanumController.driveParams(0, 0, 0);
                break;
            }
            mecanumController.driveParams(0, 0.2, 0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.SINGLE_OVERRIDE)
                .setNaturalSpeed(0.6)
                .setOverrideOneSpeed(1)
                .build();

        mecanumController = new MecanumController(hardwareMap, speedController);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
//        boolean holdingY = false;

//        Servo outtake = hardwareMap.get(Servo.class, "outtake");
//        outtake.setDirection(Servo.Direction.REVERSE);
//        outtake.setPosition(Constants.OUTTAKE_HIDDEN);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        CRServo air = hardwareMap.get(CRServo.class, "air");
        air.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor climber = hardwareMap.get(DcMotor.class, "climber");

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.OPEN_CLAW)) {
                claw.setPosition(Constants.CLAW_OPEN);
            } else if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLOSE_CLAW)) {
                claw.setPosition(Constants.CLAW_CLOSE);
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

//            if (gamepad1.y && !holdingY) {
//                claw.setPosition(Constants.CLAW_OPEN);
//                climber.setPower(0);
//                air.setPower(0);
//
//                pixelSonar();
//                claw.setPosition(Constants.CLAW_CLOSE);
//            }

//            holdingY = gamepad1.y;
        }
    }
}
