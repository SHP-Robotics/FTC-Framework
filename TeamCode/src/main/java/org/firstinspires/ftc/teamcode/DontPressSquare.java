package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotMode.DRIVING;
import static org.firstinspires.ftc.teamcode.RobotMode.INTAKE;
import static org.firstinspires.ftc.teamcode.RobotMode.OUTTAKE;

import static org.firstinspires.ftc.teamcode.HangMode.SETUP;
import static org.firstinspires.ftc.teamcode.HangMode.VIPERDOWN;
import static org.firstinspires.ftc.teamcode.HangMode.WORMBACK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.MecanumTracker;
import com.shprobotics.pestocore.drivebases.TeleOpController;

@TeleOp
public class DontPressSquare extends LinearOpMode {
    double wristPos = 0.0;
    double clawPos = 0.0;
    boolean first=true;
    RobotMode mode = RobotMode.DRIVING;
    HangMode modePosition = HangMode.SETUP;

    int moveSpeed = 2;
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
        teleOpController.resetIMU();
        mecanumTracker.reset();
        while (opModeIsActive()) {
            mecanumTracker.updateOdometry();
            teleOpController.updateSpeed(gamepad1);
            teleOpController.driveFieldCentric(gamepad1.left_stick_y * moveSpeed, -gamepad1.left_stick_x * moveSpeed, -gamepad1.right_stick_x);


            telemetry.addData("viper", viperslide.getCurrentPosition());
            telemetry.addData("wormgear", wormgear.getCurrentPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("wrist", wristPos);
            telemetry.addData("wrist", wrist.getPosition());

            telemetry.addData("moveSpeed: ", moveSpeed);
            telemetry.addData("mode: ", mode);
            telemetry.update();



            if (gamepad2.b) {
                teleOpController.resetIMU();
                mecanumTracker.reset();
            }
//            if (gamepad1.b || gamepad1.x) {
//                wormgear.setPower((gamepad1.b ? 0.6 : 0) - (gamepad1.x ? 0.6 : 0));
//            }
//            if (gamepad1.y) {
//                viperslide.setTargetPosition(2150);
//                viperslide.setPower(0.4);
//                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            if (gamepad1.a) {
//                viperslide.setTargetPosition(0);
//                viperslide.setPower(0.1);
//                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            if (gamepad2.x) {
//                viperslide.setPower(1);
//            }

//            if (gamepad2.dpad_down) {
//                viperslide.setTargetPosition(0);
//                viperslide.setPower(0.4);
//                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                wormgear.setTargetPosition(0);
//                wormgear.setPower(0.4);
//                wormgear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }






            if (gamepad1.left_bumper) {
                while (gamepad1.left_bumper) {
                }

                switch (mode) {
                    case DRIVING:
                        if (first) {
                            first=false;
                            mode = OUTTAKE;

                        }else{
                            mode = INTAKE;

                        }
                        break;
                    case INTAKE:
                        mode = OUTTAKE;
                        break;
                    case OUTTAKE:
                        mode = DRIVING;
                        break;
                }

                if (mode == INTAKE) {
                    viperslide.setTargetPosition(0);
                    viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    viperslide.setPower(0.6);

                    wormgear.setTargetPosition(-1530);
                    wormgear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormgear.setPower(0.6);

                    claw.setPosition(0.7);
                    wrist.setPosition(0.49);
                }



                if (mode == RobotMode.OUTTAKE) {
                    claw.setPosition(0.3);

                    viperslide.setTargetPosition(2175);
                    viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    viperslide.setPower(0.6);

                    wormgear.setTargetPosition(0);
                    wormgear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormgear.setPower(0.6);

                    wrist.setPosition(0.85);
                }

                if (mode == RobotMode.DRIVING) {

                    viperslide.setTargetPosition(0);
                    viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    viperslide.setPower(0.6);

                    wormgear.setTargetPosition(0);
                    wormgear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormgear.setPower(0.6);

                    claw.setPosition(0.3);
                    wrist.setPosition(0.5);
                }
            }


            //testing hang
            if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) {
                }

                switch (modePosition) {
                    case SETUP:
                        modePosition = VIPERDOWN;
                        break;
                    case VIPERDOWN:
                        modePosition = WORMBACK;
                        break;
                    case WORMBACK:
                        modePosition = SETUP;
                        break;
                }

                if (modePosition == HangMode.SETUP) {
                    viperslide.setTargetPosition(1000);
                    viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    viperslide.setPower(0.6);

                    wormgear.setTargetPosition(0);
                    wormgear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormgear.setPower(0.6);

                    claw.setPosition(0.7);
                    wrist.setPosition(0.85);
                }




                if (modePosition == HangMode.WORMBACK) {


                    wormgear.setTargetPosition(1700);
                    wormgear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wormgear.setPower(0.6);


                }
            }

            if (modePosition == HangMode.WORMBACK || modePosition == HangMode.VIPERDOWN) {
                viperslide.setTargetPosition(-100);
                viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperslide.setPower(0.8);
            }
                //end




            if (gamepad1.dpad_left) {
                clawPos += 0.05;

                claw.setPosition(0.3);
            }

            if (gamepad1.dpad_right) {
                clawPos -= 0.05;

                claw.setPosition(0.7);
            }

            if (gamepad1.dpad_up) {
                if (wristPos < 1) {
                    wristPos += 0.05;
                    wrist.setPosition(wristPos);
                }
            }

            if (gamepad1.dpad_down) {
                if (wristPos > 0) {
                    wristPos -= 0.05;
                    wrist.setPosition(wristPos);
                }

            }
        }
    }
}