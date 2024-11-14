package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.MecanumTracker;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.geometries.Vector2D;
@TeleOp
public class test extends LinearOpMode {
    boolean slideon=false;
    double wristPos=0.0;
    double clawpos=0.0;


    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        MecanumTracker mecanumTracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, mecanumTracker, hardwareMap);

        DcMotor viperslide = hardwareMap.get(DcMotor.class, "ViperSlide");
        viperslide.setDirection(DcMotorSimple.Direction.REVERSE);
        viperslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        DcMotor wormgear = hardwareMap.get(DcMotor.class, "WormGear");
        Servo claw = hardwareMap.get(Servo.class, "Claw");


        waitForStart();

        while (opModeIsActive()) {
            mecanumTracker.updateOdometry();
            Vector2D currentPosition = mecanumTracker.getCurrentPosition();
            double heading = mecanumTracker.getCurrentHeading();
            teleOpController.updateSpeed(gamepad1);
            teleOpController.driveRobotCentric(gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x);
//            teleOpController.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("x", currentPosition.getX());
            telemetry.addData("y", currentPosition.getY());
            telemetry.addData("rotation", heading);
            telemetry.addData("viper", viperslide.getCurrentPosition());
            telemetry.addData("wormgear", wormgear.getCurrentPosition());
            telemetry.addData("claw", wormgear.getCurrentPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("slideon: ", slideon);



            telemetry.update();


            if (gamepad1.b) {
                mecanumTracker.reset();
                teleOpController.resetIMU();

            }

            wormgear.setPower((gamepad1.b ? 0.5: 0) - (gamepad1.x ? 0.5: 0));

            if (gamepad1.y) {
                viperslide.setTargetPosition(2150);
                viperslide.setPower(0.4);
                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.a) {
                viperslide.setTargetPosition(0);
                viperslide.setPower(0.4);
                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

//

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
//            if (gamepad1.left_stick_y >0.7  && gamepad1.right_stick_y <-0.7) {
//                claw.setPosition(0.5);
//
//
//            }

//            if (gamepad1.x) {
//                viperslide.setPower(0);
//                wormgear.setPower(0);
//
//            }

            if (gamepad1.dpad_left) {
                clawpos+=0.05;
                claw.setPosition(clawpos);
                }

            if (gamepad1.dpad_right) {
                clawpos-=0.05;

                claw.setPosition(clawpos);
            }

            if (gamepad1.dpad_up) {
                if (wristPos<1) {
                    wristPos+=0.05;

                    wrist.setPosition(wristPos);
                }
            }

            if (gamepad1.dpad_down) {
                if (wristPos>0) {
                    wristPos-=0.05;

                    wrist.setPosition(wristPos);
                }
            }
            }

            //telemetry.addData("Motor 0:",)
        }
    }

