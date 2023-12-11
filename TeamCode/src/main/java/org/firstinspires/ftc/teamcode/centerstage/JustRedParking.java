package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class JustRedParking extends LinearOpMode {
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

            mecanumController.moveToPosition(0, 26.25, true);
            mecanumController.rotateToRadianUsingPID(Math.toRadians(90.4), Math.toRadians(0), 4);

            mecanumController.moveToPosition(6.5, 26.25,true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
            mecanumController.moveToPosition(0, 26.25, true);

        } else if (location == 2) {

            mecanumController.moveToPosition(0, 33.25, true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
            mecanumController.moveToPosition(0, 0, true);
            mecanumController.moveToPosition(48, 0, true);

        } else {

            mecanumController.moveToPosition(0, 26.25, true);
            mecanumController.rotateToRadianUsingPID(Math.toRadians(90.4), Math.toRadians(0), 4);

            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);

            mecanumController.moveToPosition(-16.5, 26.25,true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            mecanumController.moveToPosition(-18.5, 26.25, true);
        }
    }
}
