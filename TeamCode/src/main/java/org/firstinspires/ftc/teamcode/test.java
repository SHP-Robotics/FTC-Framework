package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.MecanumTracker;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Vector2D;

@TeleOp
public class test extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        ThreeWheelOdometryTracker threeWheelOdometryTracker = PestoFTCConfig.getTracker(hardwareMap);
        //MecanumTracker mecanumTracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, threeWheelOdometryTracker, hardwareMap);

        DcMotor viperslide = hardwareMap.get(DcMotor.class, "ViperSlide");
        DcMotor wormgear = hardwareMap.get(DcMotor.class, "WormGear");
        Servo claw = hardwareMap.get(Servo.class, "Claw");
        DcMotor opodL = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor opodC = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor opodR = hardwareMap.get(DcMotor.class, "backLeft");


        waitForStart();

        while (opModeIsActive()) {
            threeWheelOdometryTracker.updateOdometry();
            Vector2D currentPosition = threeWheelOdometryTracker.getCurrentPosition();
            double heading = threeWheelOdometryTracker.getCurrentHeading();

            teleOpController.updateSpeed(gamepad1);

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //viperslide.setPower(0);
            wormgear.setPower(gamepad1.right_trigger);
            wormgear.setPower(-gamepad1.left_trigger);

            //arm speed constraints
            if (gamepad1.right_stick_y > 0.5) {
                viperslide.setPower(0.5);
                }
            //if gamepad1.right_stick_y is between 10% and 50%
            else if (gamepad1.right_stick_y > 0.1 && gamepad1.right_stick_y < 0.5) {
                viperslide.setPower(0.3);
            }
            else {
                viperslide.setPower(0);
            }
            

            if (gamepad1.x) {
                    claw.setPosition(0.5);
                }
            if (gamepad1.y){
                claw.setPosition(1.0);
            }

            telemetry.addData("x", currentPosition.getX());
            telemetry.addData("y", currentPosition.getY());
            telemetry.addData("rotation", heading);
            telemetry.addData("OPOD L", opodL.getCurrentPosition());
            telemetry.addData("OPOD C", opodC.getCurrentPosition());
            telemetry.addData("OPOD R", opodR.getCurrentPosition());
            telemetry.update();


            if (gamepad1.b) {
                threeWheelOdometryTracker.reset();
                teleOpController.resetIMU();
            }
            //telemetry.addData("Motor 0:",)
        }
    }
}
