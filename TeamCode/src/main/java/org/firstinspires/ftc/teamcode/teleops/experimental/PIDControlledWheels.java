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

    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.SINGLE_OVERRIDE)
                .setNaturalSpeed(0.45)
                .setOverrideOneSpeed(1)
                .build();

        mecanumController = new MecanumController(hardwareMap, speedController);

        mecanumController.leftFront = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.leftFront)
                .setkC(0)
                .setkB(1/0.976)
//                .setkC(0.00015)
//                .setkB(0.3/0.976)
                .setGamma(0)
                .build();

        mecanumController.rightFront = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.rightFront)
                .setkC(0)
                .setkB(1/0.992)
//                .setkC(0.00015)
//                .setkB(0.3/0.992)
                .setGamma(0)
                .build();

        mecanumController.leftRear = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.leftRear)
                .setkC(0)
                .setkB(1/0.992)
//                .setkC(0.00015)
//                .setkB(0.3/0.992)
                .setGamma(0)
                .build();

        mecanumController.rightRear = new PIDControlledDcMotor.PIDControlledDcMotorBuilder(mecanumController.rightRear)
                .setkC(0)
                .setkB(1)
//                .setkC(0.00015)
//                .setkB(0.3)
                .setGamma(0)
                .build();

        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        visionSubsystem = new VisionSubsystem(hardwareMap, "pixel");

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_HIDDEN);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo air = hardwareMap.get(Servo.class, "air");
        air.setDirection(Servo.Direction.REVERSE);

        DcMotor climber = hardwareMap.get(DcMotor.class, "climber");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            mecanumController.fieldOrientedDrivePID(gamepad1);

            telemetry.addData("radians", mecanumController.getCalibratedIMUAngle());

            telemetry.addData("driveSpeed", mecanumController.getDriveSpeed());

            if (gamepad1.b) {
                mecanumController.calibrateIMUAngleOffset();
            }

            if (mecanumController.leftFront instanceof PIDControlledDcMotor) {
                telemetry.addData("velo fl", -((PIDControlledDcMotor) mecanumController.leftFront).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.rightFront instanceof PIDControlledDcMotor) {
                telemetry.addData("velo fr", -((PIDControlledDcMotor) mecanumController.rightFront).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.leftRear instanceof PIDControlledDcMotor) {
                telemetry.addData("velo rl", -((PIDControlledDcMotor) mecanumController.leftRear).getVelocity(AngleUnit.RADIANS));
            }

            if (mecanumController.rightRear instanceof PIDControlledDcMotor) {
                telemetry.addData("velo rr", -((PIDControlledDcMotor) mecanumController.rightRear).getVelocity(AngleUnit.RADIANS));
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

            air.setPosition(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.AIR_POWER) ? Constants.AIRPLANE_RELEASE: Constants.AIRPLANE_HOLD);
        }
    }
}
