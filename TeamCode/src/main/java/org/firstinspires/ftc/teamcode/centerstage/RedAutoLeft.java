package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class RedAutoLeft extends LinearOpMode {
    VisionSubsystem vision;

    public int location;

    @Override
    public void runOpMode() {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.2);
        mecanumController.setRotationSpeed(0.2);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(Constants.CLAW_CLOSE);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        vision = new VisionSubsystem(hardwareMap,"red");
        location = vision.getLocationRed();
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
            location = vision.getLocationRed();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();
        mecanumController.calibrateIMUAngleOffset();

        if (location == 1) {
            mecanumController.moveToPosition(0, 32, true);

            mecanumController.rotateToRadian(Math.toRadians(-90), Math.toRadians(0.5));

            mecanumController.moveToPosition(-15.5, 32, true);

        } else if (location == 2) {
            mecanumController.moveToPosition(0, 35.75, true);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);

            mecanumController.moveToPosition(0, 27.25, true);
            mecanumController.rotateToRadian(Math.toRadians(90), Math.toRadians(0.25));

            mecanumController.moveToPosition(-93, 27.25,true);
            mecanumController.rotateToRadian(Math.toRadians(90), Math.toRadians(0.25));

            mecanumController.moveToPosition(-93, 21.25,true);
            mecanumController.moveToPosition(-94, 21.25,true);
            outtake.setPosition(Constants.OUTTAKE_ACTIVE);
            sleep(1000);

            mecanumController.setDriveSpeed(mecanumController.getDriveSpeed()*0.3);
            mecanumController.moveToPosition(91, 21.25, true);
        } else {
            mecanumController.moveToPosition(0, 32, true);

            mecanumController.rotateToRadian(Math.toRadians(-90), Math.toRadians(0.5));

            mecanumController.moveToPosition(7.5, 32, true);
        }
    }
}
