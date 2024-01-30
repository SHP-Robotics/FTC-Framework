package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.debug.PADControlledDcMotor;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp()
public class PADControlledWheels extends LinearOpMode {
    MecanumController mecanumController;
    VisionSubsystem visionSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.SINGLE_OVERRIDE)
                .setNaturalSpeed(0.45)
                .setOverrideOneSpeed(1)
                .build();

        mecanumController = new MecanumController(hardwareMap, speedController);

        mecanumController.leftFront = new PADControlledDcMotor.PADControlledDcMotorBuilder(mecanumController.leftFront)
                .setkP(1/0.976)
                .setkD(0)
                .setGamma(0)
                .build();

        mecanumController.rightFront = new PADControlledDcMotor.PADControlledDcMotorBuilder(mecanumController.rightFront)
                .setkP(1/0.992)
                .setkD(0)
                .setGamma(0)
                .build();

        mecanumController.leftRear = new PADControlledDcMotor.PADControlledDcMotorBuilder(mecanumController.leftRear)
                .setkP(1/0.992)
                .setkD(0)
                .setGamma(0)
                .build();

        mecanumController.rightRear = new PADControlledDcMotor.PADControlledDcMotorBuilder(mecanumController.rightRear)
                .setkP(1)
                .setkD(0)
                .setGamma(0)
                .build();

        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_HIDDEN);

        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setDirection(Servo.Direction.REVERSE);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        CRServo air = hardwareMap.get(CRServo.class, "air");
        air.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor climber = hardwareMap.get(DcMotor.class, "climber");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            mecanumController.fieldOrientedDrivePID(gamepad1);

            telemetry.addData("radians", mecanumController.getCalibratedIMUAngle());

            telemetry.addData("driveSpeed", mecanumController.getDriveSpeed());

            if (gamepad1.b) {
                mecanumController.calibrateIMUAngleOffset();
            }

            if (mecanumController.leftFront instanceof PADControlledDcMotor) {
                telemetry.addData("velo fl", -((PADControlledDcMotor) mecanumController.leftFront).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.rightFront instanceof PADControlledDcMotor) {
                telemetry.addData("velo fr", -((PADControlledDcMotor) mecanumController.rightFront).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.leftRear instanceof PADControlledDcMotor) {
                telemetry.addData("velo rl", -((PADControlledDcMotor) mecanumController.leftRear).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.rightRear instanceof PADControlledDcMotor) {
                telemetry.addData("velo rr", -((PADControlledDcMotor) mecanumController.rightRear).getVelocity(AngleUnit.RADIANS));
            }

            telemetry.update();

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
            cameraServo.setPosition(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CAMERA_POWER) ? Constants.CameraMode.FACING_CLAW.getPosition() : Constants.CameraMode.FACING_TEAM_PROP.getPosition());
        }
    }
}
