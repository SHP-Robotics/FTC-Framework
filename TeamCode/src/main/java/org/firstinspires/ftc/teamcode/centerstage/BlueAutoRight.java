package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class BlueAutoRight extends LinearOpMode {
    VisionSubsystem vision;

    public int location;

    @Override
    public void runOpMode() {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.25);
        mecanumController.setRotationSpeed(0.2);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(Constants.CLAW_CLOSE);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        vision = new VisionSubsystem(hardwareMap,"blue");
        location = vision.getLocationBlue();
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
            location = vision.getLocationBlue();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();
        mecanumController.calibrateIMUAngleOffset();

        if (location == 3) {

            mecanumController.moveToPosition(0, 26.25, true);
            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);

            mecanumController.moveToPosition(-6.5, 26.25,true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);

            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            mecanumController.moveToPosition(86.5, 26.25,true);
            mecanumController.moveToPosition(88.5, 26.25,true);
            mecanumController.moveToPosition(88.5, 24.25,true);

            //

            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            mecanumController.moveToPosition(89.5, 24.25,true);

            outtake.setPosition(Constants.OUTTAKE_ACTIVE);
            sleep(1000);

            mecanumController.setDriveSpeed(mecanumController.getDriveSpeed()*0.3);
            mecanumController.moveToPosition(84.5, 24.25, true);

        } else if (location == 2) {

            mecanumController.moveToPosition(0, 33.25, true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);

            mecanumController.moveToPosition(0, 26.25, true);
            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);

            mecanumController.moveToPosition(88.5, 26.25,true);
            mecanumController.moveToPosition(88.5, 35.25,true);

            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            mecanumController.moveToPosition(89.5, 35.25,true);

            outtake.setPosition(Constants.OUTTAKE_ACTIVE);
            sleep(1000);

            mecanumController.setDriveSpeed(mecanumController.getDriveSpeed()*0.3);
            mecanumController.moveToPosition(84.5, 35.25, true);

        } else {

            mecanumController.moveToPosition(0, 26.25, true);
            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);

            mecanumController.moveToPosition(16.5, 26.25,true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);

            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            mecanumController.moveToPosition(88.5, 26.25,true);
            mecanumController.moveToPosition(88.5, 40.25,true);

            //

            mecanumController.moveToPosition(88.5, 40.25,true);


            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            mecanumController.moveToPosition(89.5, 40.25,true);

            outtake.setPosition(Constants.OUTTAKE_ACTIVE);
            sleep(1000);

            mecanumController.setDriveSpeed(mecanumController.getDriveSpeed()*0.3);
            mecanumController.moveToPosition(82.5, 40.25, true);

        }
    }
}