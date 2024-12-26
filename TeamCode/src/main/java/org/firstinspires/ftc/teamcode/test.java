package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

//@TeleOp
//public class test extends LinearOpMode {
//
//    private DcMotor wormGearMotor;
//    private DcMotor strongArmMotor;
//
//    private DcMotor strongArm2Motor;
//    private DcMotor viperslide;
//    private Servo claw;
//    private Servo spinner;
//
////    private DistanceSensor distanceSensor;
//
//    private void threewheelOdometryTracker() {
//    }
//
//    @Override
//    public void runOpMode() {
//        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
//        ThreeWheelOdometryTracker threeWheelOdometryTracker = PestoFTCConfig.getTracker(hardwareMap);
//        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, threeWheelOdometryTracker, hardwareMap);
//
//        wormGearMotor = hardwareMap.get(DcMotor.class, "wormGear");
//        wormGearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        wormGearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        wormGearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        strongArmMotor = hardwareMap.get(DcMotor.class, "strongArm");
//        strongArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        strongArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        strongArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        strongArm2Motor = hardwareMap.get(DcMotor.class, "strongArm2");
//        strongArm2Motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        strongArm2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        strongArm2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        viperslide = hardwareMap.get(DcMotor.class, "arm");
//        viperslide.setDirection(DcMotorSimple.Direction.FORWARD);
//        viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        claw = hardwareMap.get(Servo.class, "Claw");
//        claw.setDirection(Servo.Direction.FORWARD);
//
//        spinner = hardwareMap.get(Servo.class, "Spinner");
//        spinner.setDirection(Servo.Direction.FORWARD);
//
////        hardwareMap.get(ColorSensor.class, "LED Indicator");
////        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            threeWheelOdometryTracker.update();
//            threewheelOdometryTracker();
//            Vector2D currentPosition = threeWheelOdometryTracker.getCurrentPosition().asVector();
//            Pose2D heading = threeWheelOdometryTracker.getCurrentPosition();
//
//            updateTelemetry(telemetry);
//            teleOpController.updateSpeed(gamepad1);
//            teleOpController.updateSpeed(gamepad2);
//
//            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//
//            if (gamepad1.right_trigger > 0.2) {
//                wormGearMotor.setPower(gamepad1.right_trigger);
//            } else if (gamepad1.left_trigger > 0.2) {
//                wormGearMotor.setPower(-gamepad1.left_trigger);
//            } else {
//                wormGearMotor.setPower(0.0);
//            }
//
//            if (gamepad1.right_trigger > 0) {
//                wormGearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                wormGearMotor.setTargetPosition(-1850);
//                wormGearMotor.setPower(1.0);
//            }
////                while (viperslide.isBusy()) {
////                }
////                wormGearMotor.setPower(0.0);
////                wormGearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            if (gamepad2.right_stick_y > 0.2) {
//                strongArmMotor.setPower(gamepad2.right_stick_y);
//            } else if (gamepad2.right_stick_y < 0.2) {
//                strongArmMotor.setPower(gamepad2.right_stick_y);
//            } else {
//                strongArmMotor.setPower(0.0);
//            }
//
//            if (gamepad2.left_stick_y > 0.2) {
//                strongArm2Motor.setPower(gamepad2.left_stick_y);
//            } else if (gamepad2.left_stick_y < 0.2) {
//                strongArm2Motor.setPower(gamepad2.left_stick_y);
//            } else {
//                strongArm2Motor.setPower(0.0);
//            }
//
//            if (gamepad1.right_stick_y > 0.2 && !(viperslide.getCurrentPosition() < -1500 && wormGearMotor.getCurrentPosition() > 15000)) {
//                viperslide.setPower(gamepad1.right_stick_y);
//            } else if (gamepad1.right_stick_y < 0.2) {
//                viperslide.setPower(gamepad1.right_stick_y);
//            } else {
//                viperslide.setPower(0.0);
//            }
//
//            if (gamepad1.dpad_up) {
//                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                viperslide.setTargetPosition(-2375);
//                viperslide.setPower(1.0);
//                viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            if (gamepad1.dpad_down) {
//                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                viperslide.setTargetPosition(145);
//                viperslide.setPower(1.0);
//                viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                while (viperslide.isBusy()) {
//                    viperslide.getCurrentPosition();
//                }
//            }
//
//            if (gamepad1.x) {
//                claw.setPosition(0.75);
//            }
//            if (gamepad1.y) {
//                claw.setPosition(1.0);
//            }
//
//            if (gamepad2.x) {
//                strongArmMotor.setTargetPosition(-1104);
////                strongArm2Motor.setTargetPosition(INSERT HERE...);
//                wormGearMotor.setTargetPosition(908);
//                claw.setPosition(1.0);
//            }
//
//            if (gamepad1.b) {
//                threeWheelOdometryTracker.reset();
//                teleOpController.resetIMU();
//                telemetry.update();
//            }
//
////            if (gamepad2.right_trigger > 0) {
////                spinner.setDirection(Servo.Direction.FORWARD);
////            } else if (gamepad2.left_trigger > 0) {
////                spinner.setDirection(Servo.Direction.REVERSE);
////            } else {
////                spinner.setPosition(0);
////            }
//
//
////    private void updateLEDColor() {
//        }
//
//
////    public static class test {
////    private ColorSensor rgbIndicator;
////    private DistanceSensor distanceSensor;
//
////    public void init (HardwareMap hardwareMap) {
////        rgbIndicator = hardwareMap.get(ColorSensor.class, "LED Indicator");
////        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
////    }
//
////    public void updateLEDColor() {
////        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
//
////        if (distanceInches == 5.0) {
////            setRGBColor(0.0, 1.0, 0.0);
////        } else {
////            setRGBColor(1.0, 0.0, 0.0);
////        }
////    }
////
////    public void setRGBColor(double red, double green, double blue) {
////        if (red >= 0.0 && red <= 1.0 && green >= 0.0 && green <= 1.0 && blue >= 0.0 && blue <= 1.0) {
////            //if red, green, and blue are all between 0 and 1, then set the colors accordingly
////            rgbIndicator.argb();
////            rgbIndicator.argb();
////            rgbIndicator.argb();
////        } else {
////            throw new IllegalArgumentException("Color values must be between 0.0 and 1.0");
////        }
////    }
////
////    public void updateTelemetry(Telemetry telemetry) {
////        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
////        telemetry.addData("Distance (inches)", distanceInches);
////        telemetry.update();
////    }
////
////    public void loop (Telemetry telemetry) {
////        updateLEDColor();
////        updateTelemetry(telemetry);
////    }
////
////
////    double distanceInches; {
////        distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
////    }
//
////    DcMotor threewheelOdometryTracker = null;
//
////    public void updateTelemetry(Telemetry telemetry) {
//
//
//        // Assuming getX() and getY() are methods that return the respective coordinates
//            telemetry.addData("X", getX());
//            telemetry.addData("Y", getY());
//            telemetry.addData("Rotation", geheading());
//            telemetry.addData("WormGear", wormGearMotor.getCurrentPosition());
//            telemetry.addData("StrongArm", strongArmMotor.getCurrentPosition());
//            telemetry.addData("ViperSlide", viperslide.getCurrentPosition());
//            telemetry.addData("Claw", claw.getPosition());
//            telemetry.addData("Spinner", spinner.getPosition());
//            telemetry.addData("Odometry", threeWheelOdometryTracker.getCurrentPosition());
////        telemetry.addData("Distance", distanceInches);
////        telemetry.addData("LED Color", hardwareMap.colorSensor);
//            telemetry.update();
//        }
//    private String getY() {
//        return null;
//    }
//    private String getX() {
//        return null;
//    }
//}


//THIS IS THE NEW STUFF - I WENT BACK AND STARTED FROM THE ORIGINAL ONE
@TeleOp
public class test extends LinearOpMode {
@Override
public void runOpMode() {
    MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
    ThreeWheelOdometryTracker threeWheelOdometryTracker = PestoFTCConfig.getTracker(hardwareMap);
    ThreeWheelOdometryTracker mecanumTracker = PestoFTCConfig.getTracker(hardwareMap);
    TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, threeWheelOdometryTracker, hardwareMap);
//
//    DcMotor viperslide = hardwareMap.get(DcMotor.class, "viperSlide");
//    DcMotor wormgear = hardwareMap.get(DcMotor.class, "wormGear");
//    DcMotor strongArm = hardwareMap.get(DcMotor.class, "strongArm");
//    DcMotor strongArm2 = hardwareMap.get(DcMotor.class, "strongArm2");
//    Servo claw = hardwareMap.get(Servo.class, "Claw");
//    Servo spinner = hardwareMap.get(Servo.class, "Spinner");
    ThreeWheelOdometryTracker opodL = hardwareMap.get(ThreeWheelOdometryTracker.class, "frontLeft");
    ThreeWheelOdometryTracker opodC = hardwareMap.get(ThreeWheelOdometryTracker.class, "backCenter");
    ThreeWheelOdometryTracker opodR = hardwareMap.get(ThreeWheelOdometryTracker.class, "frontRight");

    waitForStart();

    while (opModeIsActive()) {
        threeWheelOdometryTracker.update();
        Vector2D currentPosition = threeWheelOdometryTracker.getCurrentPosition().asVector();
        Pose2D heading = threeWheelOdometryTracker.getCurrentPosition();

        teleOpController.updateSpeed(gamepad1);

        teleOpController.driveFieldCentric(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

//        viperslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        viperslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//
//        wormgear.setPower(gamepad1.right_trigger);
//        wormgear.setPower(-gamepad1.left_trigger);
//
//        //arm speed constraints
//        if (gamepad1.right_stick_y > 0 && gamepad1.right_stick_y < 0.5) {
//            viperslide.setPower(0.5);
//        }
//        else if (gamepad1.right_stick_y < 0.5) {
//            viperslide.setPower(1);
//        }
//        else if (gamepad1.right_stick_y < 0 && gamepad1.right_stick_y > -0.5) {
//            viperslide.setPower(-0.5);
//        }
//        else
//            viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//
//        if (gamepad1.x) {
//            claw.setPosition(0.75);
//        }
//        if (gamepad1.y) {
//            claw.setPosition(1);
//        }
//
//       if (gamepad2.left_stick_y > 0)
//            strongArm.setPower(gamepad2.left_stick_y);
//        else if (gamepad2.left_stick_y < 0)
//            strongArm.setPower(gamepad2.left_stick_y);
//        else
//            strongArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        if (gamepad2.left_stick_y > 0)
//            strongArm2.setPower(gamepad2.right_stick_y);
//        else if (gamepad2.right_stick_y < 0)
//            strongArm2.setPower(gamepad2.right_stick_y);
//        else
//            strongArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        if (gamepad2.right_trigger > 0)
//            spinner.setDirection(Servo.Direction.FORWARD);
//        else if (gamepad2.left_trigger > 0)
//            spinner.setDirection(Servo.Direction.FORWARD);
//        else
//            spinner.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("x", currentPosition.getX());
        telemetry.addData("y", currentPosition.getY());
        telemetry.addData("Rotation/Heading", heading);
        telemetry.addData("frontLeft", opodL.getCurrentPosition());
        telemetry.addData("backCenter", opodC.getCurrentPosition());
        telemetry.addData("frontRight", opodR.getCurrentPosition());
//        telemetry.addData("viperSlide: ", viperslide.getCurrentPosition());
//        telemetry.addData("Spinner: ", spinner.getPosition());
//        telemetry.addData("Claw: ", claw.getPosition());
//        telemetry.addData("strongArm: ", strongArm.getCurrentPosition());
//        telemetry.addData("strongArm2: ", strongArm2.getCurrentPosition());
//        telemetry.addData("wormGear: ", wormgear.getCurrentPosition());
//        telemetry.update();


            if (gamepad1.b) {
                threeWheelOdometryTracker.reset();
                teleOpController.resetIMU();
            }

//            if (gamepad2.x) {
//                strongArm.setPower(1);
//                strongArm2.setPower(1);
//                claw.setPosition(1);
//             }
        }
    }
}