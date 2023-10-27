package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.*;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

@TeleOp(name = "Mecanum Wheel Drive")
public class MecanumWheelDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
         boolean gearDown = false;
         boolean gearUp = true;

        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.7);

        waitForStart();
        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.GEAR_UP)) {
                if (!gearUp) {
                    mecanumController.gearUp();
                }
                gearUp = true;
            } else {
                gearUp = false;
            }

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.GEAR_DOWN)) {
                if (!gearDown) {
                    mecanumController.gearDown();
                }
                gearDown = true;
            } else {
                gearDown = false;
            }
        }
    }
}
