package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumController;
import org.firstinspires.ftc.teamcode.config.DrivingConfiguration;

@Disabled
@TeleOp(name = "Field Oriented Mecanum Wheel Drive")
public class FieldOrientedMecanumWheelDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.calibrateIMUAngleOffset();
        mecanumController.setDriveSpeed(1);

        waitForStart();
        while (opModeIsActive()) {
            mecanumController.fieldOrientedDrive(gamepad1);
            telemetry.addData("radians clockwise", mecanumController.getCalibratedIMUAngle());
            telemetry.update();

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.RESET_IMU)) {
                // I wonder if this will reset the IMU if the Yaw is off after a collision
                // Further testing required
                // mecanumController.initIMU(hardwareMap);
                mecanumController.calibrateIMUAngleOffset();
            }
        }
    }
}
