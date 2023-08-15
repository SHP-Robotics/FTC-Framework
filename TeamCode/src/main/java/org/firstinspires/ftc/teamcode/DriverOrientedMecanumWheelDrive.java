package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generals.*;

@TeleOp(name = "Driver Oriented Mecanum Wheel Drive")
public class DriverOrientedMecanumWheelDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, RuntimeType.DRIVER_CONTROLLED_TELEOP);
        mecanumController.calibrateIMUAngleOffset();

        waitForStart();
        while (opModeIsActive()) {
            mecanumController.driverOrientedDrive(gamepad1);

            if (gamepad1.b) {
                mecanumController.initIMU(hardwareMap);
                mecanumController.calibrateIMUAngleOffset();
            }
        }
    }
}
