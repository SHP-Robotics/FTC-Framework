package org.firstinspires.ftc.teamcode.autos.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Disabled
@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class JustBlue extends LinearOpMode {
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
            mecanumController.moveToPosition(0, 26.25, true);

        } else if (location == 2) {

            mecanumController.moveToPosition(0, 33.25, true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
            mecanumController.moveToPosition(0, 27.25, true);

        } else {

            mecanumController.moveToPosition(0, 26.25, true);
            mecanumController.rotateToRadianUsingPID(Math.toRadians(-90.4), Math.toRadians(0), 4);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);

            mecanumController.moveToPosition(15.5, 26.25,true);
            sleep(100);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            mecanumController.moveToPosition(18.5, 26.25,true);

        }
    }
}
