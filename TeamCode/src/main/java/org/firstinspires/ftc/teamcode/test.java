package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
public class test extends LinearOpMode {

    private DcMotor wormGearMotor;
    private DcMotor strongArmMotor;
    private DcMotor viperslide;
    private Servo claw;

    private DistanceSensor distanceSensor;

    public double getX;
    public double getY;

    private void threewheelOdometryTracker() {
    }

    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        ThreeWheelOdometryTracker threeWheelOdometryTracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, threeWheelOdometryTracker, hardwareMap);

        wormGearMotor = hardwareMap.get(DcMotor.class, "wormGear");
        wormGearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wormGearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormGearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        strongArmMotor = hardwareMap.get(DcMotor.class, "strongArm");
        strongArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        strongArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strongArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viperslide = hardwareMap.get(DcMotor.class, "arm");
        viperslide.setDirection(DcMotorSimple.Direction.FORWARD);
        viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "Claw");
        claw.setDirection(Servo.Direction.FORWARD);

        ColorSensor rgbIndicator = hardwareMap.get(ColorSensor.class, "LED Indicator");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");

        waitForStart();

        while (opModeIsActive()) {
            threeWheelOdometryTracker.update();
            threewheelOdometryTracker();
            Vector2D currentPosition = threeWheelOdometryTracker.getCurrentPosition().asVector();
            Pose2D heading = threeWheelOdometryTracker.getCurrentPosition();

            teleOpController.updateSpeed(gamepad1);
            teleOpController.updateSpeed(gamepad2);

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.right_trigger > 0.2) {
                wormGearMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.2) {
                wormGearMotor.setPower(-gamepad1.left_trigger);
            } else {
                wormGearMotor.setPower(0.0);
            }

            if (gamepad1.right_trigger > 0) {
                wormGearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wormGearMotor.setTargetPosition(-1850);
                wormGearMotor.setPower(1.0);
            }
//                while (viperslide.isBusy()) {
//                }
//                wormGearMotor.setPower(0.0);
//                wormGearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            if (gamepad2.right_stick_y > 0.2) {
                strongArmMotor.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < 0.2) {
                strongArmMotor.setPower(gamepad2.right_stick_y);
            } else {
                strongArmMotor.setPower(0.0);
            }

            if (gamepad1.right_stick_y > 0.2 && !(viperslide.getCurrentPosition() < -1500 && wormGearMotor.getCurrentPosition() > 15000)) {
                viperslide.setPower(gamepad1.right_stick_y);
            } else if (gamepad1.right_stick_y < 0.2) {
                viperslide.setPower(gamepad1.right_stick_y);
            } else {
                viperslide.setPower(0.0);
            }

            if (gamepad1.dpad_up) {
                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperslide.setTargetPosition(-2375);
                viperslide.setPower(1.0);
                viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.dpad_down) {
                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperslide.setTargetPosition(145);
                viperslide.setPower(1.0);
                viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                while (viperslide.isBusy()) {
                    viperslide.getCurrentPosition();
                }
            }

            if (gamepad1.x) {
                claw.setPosition(0.75);
            }
            if (gamepad1.y) {
                claw.setPosition(1.0);
            }

            if (gamepad2.x) {
                strongArmMotor.setTargetPosition(-1104);
                wormGearMotor.setTargetPosition(908);
                claw.setPosition(1.0);
            }

            updateLEDColor();
            updateTelemetry(telemetry);

            if (gamepad1.b) {
                threeWheelOdometryTracker.reset();
                teleOpController.resetIMU();
                telemetry.update();
            }
        }
    }

    private void updateLEDColor() {
    }


    public static class MyRobot {
    private ColorSensor rgbIndicator;
    private DistanceSensor distanceSensor;

    public void init (HardwareMap hardwareMap) {
        rgbIndicator = hardwareMap.get(ColorSensor.class, "rgbIndicator");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public void updateLEDColor() {
        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);

        if (distanceInches == 5.0) {
            setRGBColor(0.0, 1.0, 0.0);
        } else {
            setRGBColor(1.0, 0.0, 0.0);
        }
    }

    public void setRGBColor(double red, double green, double blue) {
        if (red >= 0.0 && red <= 1.0 && green >= 0.0 && green <= 1.0 && blue >= 0.0 && blue <= 1.0) {
            //if red, green, and blue are all between 0 and 1, then set the colors accordingly
            rgbIndicator.argb();
            rgbIndicator.argb();
            rgbIndicator.argb();
        } else {
            throw new IllegalArgumentException("Color values must be between 0.0 and 1.0");
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        // Get the current distance
        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        // Update telemetry with the distance
        telemetry.addData("Distance (inches)", distanceInches);
        telemetry.update();
    }

    public void loop (Telemetry telemetry) {
        updateLEDColor();
        updateTelemetry(telemetry);
    }
}
    DcMotor threewheelOdometryTracker = null;

    public void updateTelemetry(Telemetry telemetry) {
        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("X", getX); // Ensure getX() is defined
        telemetry.addData("Y", getY); // Ensure getY() is defined
        telemetry.addData("Heading/Rotation", ""); // Add actual heading/rotation data
        telemetry.addData("WormGear", wormGearMotor.getCurrentPosition());
        telemetry.addData("StrongArm", strongArmMotor.getCurrentPosition());
        telemetry.addData("ViperSlide", viperslide.getCurrentPosition());
        telemetry.addData("Claw", claw.getPosition());
        telemetry.addData("Odometry", threewheelOdometryTracker.getCurrentPosition());
        telemetry.addData("Distance (inches)", distanceInches);
        telemetry.update();
    }
}