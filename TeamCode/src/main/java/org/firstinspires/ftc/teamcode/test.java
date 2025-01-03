package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Vector2D;

@TeleOp
public class test extends LinearOpMode {

    private DcMotor wormGearMotor;
    private DcMotor strongArmMotor;
    private DcMotor strongArm2Motor;
    private DcMotor viperslide;
    private Servo claw;
    private Servo spinner;
    private DcMotor opodL;
    private DcMotor opodC;
    private DcMotor opodR;
    private DistanceSensor distanceSensor;
    private ColorSensor LEDIndicator;



    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        ThreeWheelOdometryTracker threeWheelOdometryTracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, threeWheelOdometryTracker, hardwareMap);
        threeWheelOdometryTracker.update();
//            threewheelOdometryTracker();
//            Vector2D currentPosition = threeWheelOdometryTracker.getCurrentPosition().asVector();
//            Pose2D heading = threeWheelOdometryTracker.getCurrentPosition();

        wormGearMotor = hardwareMap.get(DcMotor.class, "wormGear");
        wormGearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wormGearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormGearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        strongArmMotor = hardwareMap.get(DcMotor.class, "strongArm");
        strongArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        strongArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strongArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        strongArm2Motor = hardwareMap.get(DcMotor.class, "strongArm2");
        strongArm2Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        strongArm2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strongArm2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viperslide = hardwareMap.get(DcMotor.class, "viperSlide");
        viperslide.setDirection(DcMotorSimple.Direction.FORWARD);
        viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "Claw");
        claw.setDirection(Servo.Direction.FORWARD);

        spinner = hardwareMap.get(Servo.class, "Spinner");
        spinner.setDirection(Servo.Direction.FORWARD);

        ThreeWheelOdometryTracker opodL = hardwareMap.get(ThreeWheelOdometryTracker.class, "frontLeft");
        ThreeWheelOdometryTracker opodC = hardwareMap.get(ThreeWheelOdometryTracker.class, "backCenter");
        ThreeWheelOdometryTracker opodR = hardwareMap.get(ThreeWheelOdometryTracker.class, "frontRight");

        LEDIndicator = hardwareMap.get(ColorSensor.class, "LED Indicator");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");

        waitForStart();

        while (opModeIsActive()) {
            threeWheelOdometryTracker.update();
            Vector2D currentPosition = threeWheelOdometryTracker.getCurrentPosition().asVector();
//            double heading = ThreeWheelOdometryTracker.getCurrentHeading();

            teleOpController.updateSpeed(gamepad1);
            teleOpController.updateSpeed(gamepad2);

            teleOpController.driveFieldCentric(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y);

            //WormGear Code
            if (gamepad1.right_trigger > 0.2) {
                wormGearMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.2) {
                wormGearMotor.setPower(-gamepad1.left_trigger);
            } else {
                wormGearMotor.setPower(0.0);
            }

            //Move to hanging positions
            if (gamepad2.dpad_up) {
                viperslide.setTargetPosition(-5447);
                wormGearMotor.setTargetPosition(-1916);
                strongArmMotor.setTargetPosition(-3636);
            }

            // StrongArm Code
            if (gamepad2.right_stick_y>0.2){
                strongArmMotor.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y<0.2) {
                strongArmMotor.setPower(gamepad2.right_stick_y);
            } else strongArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

           //StrongArm2 Code
            if (gamepad2.left_stick_y > 0.2){
                strongArm2Motor.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.left_stick_y < 0.2) {
                strongArmMotor.setPower(gamepad2.left_stick_y);
            } else strongArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            //Viper Slide Code
            if (gamepad1.right_stick_y>0.2 && !(viperslide.getCurrentPosition()<-1500 && wormGearMotor.getCurrentPosition()>15000)){
                viperslide.setPower(gamepad1.right_stick_y);
            } else if (gamepad1.right_stick_y<0.2) {
                viperslide.setPower(gamepad1.right_stick_y);
            }
            else {
                viperslide.setPower(0.0);
            }

            //Claw
            if (gamepad1.x){
                claw.setPosition(0.75);
            }
            if (gamepad1.y){
                claw.setPosition(1.0);
            }

            telemetry.addData("X", currentPosition.getX());
            telemetry.addData("Y", currentPosition.getY());
//            telemetry.addData("Rotation", heading);
            telemetry.addData("IMU", currentPosition);
            telemetry.addData("opodL", opodL.getCurrentPosition());
            telemetry.addData("opodC", opodC.getCurrentPosition());
            telemetry.addData("opodR", opodR.getCurrentPosition());
            telemetry.addData("WormGear", wormGearMotor.getCurrentPosition());
            telemetry.addData("StrongArm", strongArmMotor.getCurrentPosition());
            telemetry.addData("strongArm2", strongArm2Motor.getCurrentPosition());
            telemetry.addData("ViperSlide", viperslide.getCurrentPosition());
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("Spinner", spinner.getPosition());
            telemetry.update();

            if (gamepad1.b) {
                threeWheelOdometryTracker.reset();
                teleOpController.resetIMU();
                teleOpController.updateSpeed(gamepad1);
                teleOpController.updateSpeed(gamepad2);
            }

        }
    }
}

//12/7 Comp Code