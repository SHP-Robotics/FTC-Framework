package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.debug.PIDControlledDcMotor;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp()
public class PIDControlledWheels extends LinearOpMode {
    MecanumController mecanumController;
    VisionSubsystem visionSubsystem;

//    private final String soundPath = "/sdcard/FIRST/blocks/sounds";
//    private final File soundFile = new File(soundPath + "/Holy Moley.wav");

//    final double minimumPixelMass = 0.2;

//    public void pixelSonar() {
//        while (gamepad1.y && opModeIsActive() && !isStopRequested()) {
//            if (visionSubsystem.getPixelMass() > minimumPixelMass) {
//                mecanumController.driveParams(0, 0, 0);
//                break;
//            }
//            mecanumController.driveParams(0, 0.2, 0);
//        }
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.PID_CONTROLLED_WITH_OVERRIDE)
                .setNaturalSpeed(0.6)
                .setOverrideOneSpeed(1)
                .build();

        mecanumController = new MecanumController(hardwareMap, speedController);

        mecanumController.leftFront = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.leftFront)
                .setkP(1)
                .setkD(-0.005)
                .setGamma(0.1)
                .build();

        mecanumController.rightFront = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.rightFront)
//                .setkP(0.985)
                .setkP(1)
                .setkD(-0.005)
                .setGamma(0.1)
                .build();

        mecanumController.leftRear = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.leftRear)
                .setkP(1)
                .setkD(-0.005)
                .setGamma(0.1)
                .build();

        mecanumController.rightRear = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.rightRear)
                .setkP(1)
                .setkD(-0.005)
                .setGamma(0.1)
                .build();

        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");
//        boolean holdingY = false;

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_HIDDEN);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        CRServo air = hardwareMap.get(CRServo.class, "air");
        air.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor climber = hardwareMap.get(DcMotor.class, "climber");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            mecanumController.drive(gamepad1);

//            telemetry.addData("radians", mecanumController.getCalibratedIMUAngle());

//            if (gamepad1.b) {
//                mecanumController.calibrateIMUAngleOffset();
//            }

            if (mecanumController.leftFront instanceof PIDControlledDcMotor) {
                telemetry.addData("velo fl", ((PIDControlledDcMotor) mecanumController.leftFront).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.rightFront instanceof PIDControlledDcMotor) {
                telemetry.addData("velo fr", ((PIDControlledDcMotor) mecanumController.rightFront).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.leftRear instanceof PIDControlledDcMotor) {
                telemetry.addData("velo rl", ((PIDControlledDcMotor) mecanumController.leftRear).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.rightRear instanceof PIDControlledDcMotor) {
                telemetry.addData("velo rr", ((PIDControlledDcMotor) mecanumController.rightRear).getVelocity(AngleUnit.RADIANS));
            }

            telemetry.update();

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.OPEN_CLAW)) {
                claw.setPosition(Constants.CLAW_OPEN);
//                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundFile);
            } else if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLOSE_CLAW)) {
                claw.setPosition(Constants.CLAW_CLOSE);
//                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundFile);
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