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
        mecanumController.setDriveSpeed(0.2);
        mecanumController.setRotationSpeed(0.2);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(Constants.CLAW_CLOSE);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);

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

        if (location == 1) {
            mecanumController.moveInches(24, 24, 24, 24, true);

            mecanumController.rotateToRadian(Math.toRadians(90), Math.toRadians(0.5));
        } else if (location == 2) {
            mecanumController.moveInches(35.75, 35.75, 35.75, 35.75, true);
            claw.setPosition(Constants.CLAW_OPEN);
            sleep(200);
            outtake.setPosition(Constants.OUTTAKE_NEUTRAL);

            mecanumController.moveInches(-8.5, -8.5, -8.5, -8.5, true);
            mecanumController.rotateToRadian(Math.toRadians(-90), Math.toRadians(0.5));
            mecanumController.moveInches(-93, -93, -93, -93, true);
            outtake.setPosition(Constants.OUTTAKE_ACTIVE);
            sleep(1000);
            mecanumController.setDriveSpeed(mecanumController.getDriveSpeed()*0.3);
            mecanumController.moveInches(3, 3, 3, 3, true);
        } else {
            mecanumController.moveInches(24, 24, 24, 24, true);

            mecanumController.rotateToRadian(Math.toRadians(90), Math.toRadians(0.5));
        }
    }
}
