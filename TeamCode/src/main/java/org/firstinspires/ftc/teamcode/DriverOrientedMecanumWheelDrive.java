package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generals.*;

@TeleOp(name = "Driver Oriented Mecanum Wheel Drive")
public class DriverOrientedMecanumWheelDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean gearDown = false;
        boolean gearUp = false;

        MecanumController mecanumController = new MecanumController(hardwareMap, RuntimeType.DRIVER_CONTROLLED_TELEOP);
        mecanumController.calibrateIMUAngleOffset();
        mecanumController.setDriveSpeed(0.7);

        waitForStart();
        while (opModeIsActive()) {
            mecanumController.driverOrientedDrive(gamepad1);
            telemetry.addData("radians clockwise", mecanumController.getCalibratedIMUAngle());
            telemetry.update();

            if (gamepad1.b) {
                mecanumController.initIMU(hardwareMap);
                mecanumController.calibrateIMUAngleOffset();
            }

            if (gamepad1.y) {
                if (!gearUp) {
                    mecanumController.gearUp();
                }
                gearUp = true;
            } else {
                gearUp = false;
            }

            if (gamepad1.a) {
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
