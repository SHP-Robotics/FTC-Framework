package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.MecanumTracker;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.geometries.Vector2D;

@TeleOp
public class test extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        MecanumTracker mecanumTracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, mecanumTracker, hardwareMap);

        DcMotor viperslide = hardwareMap.get(DcMotor.class, "ViperSlide");
        DcMotor wormgear = hardwareMap.get(DcMotor.class, "WormGear");
        Servo claw = hardwareMap.get(Servo.class, "Claw");


        waitForStart();

        while (opModeIsActive()) {
            mecanumTracker.updateOdometry();
            Vector2D currentPosition = mecanumTracker.getCurrentPosition();
            double heading = mecanumTracker.getCurrentHeading();

            teleOpController.updateSpeed(gamepad1);

//            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            teleOpController.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("x", currentPosition.getX());
            telemetry.addData("y", currentPosition.getY());
            telemetry.addData("rotation", heading);
            telemetry.update();


            if (gamepad1.b) {
                mecanumTracker.reset();
                teleOpController.resetIMU();

            }

            viperslide.setPower(0);
//            wormgear.setPower((gamepad1.right_bumper ? 1: 0) - (gamepad1.left_bumper ? 1: 0));
            wormgear.setPower((gamepad1.right_trigger)- (gamepad1.left_trigger)/5);
            viperslide.setPower(gamepad1.right_stick_y/10);
            //arm speed constraints
//            if (gamepad1.right_stick_y > 0.5 && viperslide.getCurrentPosition() > 0
//                    && viperslide.getCurrentPosition() < 2000) {
//
//
//            }
            //if gamepad1.right_stick_y is between 10% and 50%
//            if (gamepad1.right_stick_y > 0.1 && gamepad1.right_stick_y < 0.5
//                    && viperslide.getCurrentPosition() > 0
//                    && viperslide.getCurrentPosition() < 2000) {
//                viperslide.setPower(0.3);
//
//            }

            //if gamepad1.right_stick_y is between -10% and -50%
//            if (gamepad1.right_stick_y < -0.1 && gamepad1.right_stick_y > -0.5
//                    && viperslide.getCurrentPosition() > 0
//                    && viperslide.getCurrentPosition() < 2000) {
//                viperslide.setPower(-0.3);

//                if (gamepad1.right_trigger > 0.4) {
//
//                    wormgear.setPower(0.3);
//
//                }

                if (gamepad1.dpad_left) {
                    claw.setPosition(0.5);
                }

            if (gamepad1.dpad_right) {
                claw.setPosition(-0.5);
            }
            }

            //telemetry.addData("Motor 0:",)
        }
    }

