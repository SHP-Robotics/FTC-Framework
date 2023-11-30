package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.3);

        telemetry.addLine("L + Bozo + Ratio");
        telemetry.update();

        waitForStart();

        // TODO: calibrate wheel encoder ticks to inches
        mecanumController.moveToPosition(0, 24, true);
        // mecanumController.moveToPosition(24, 0, true);
        // TODO: calibrate IMU direction in MecanumController class
        // mecanumController.rotateToRadian(Math.toRadians(90), Math.toRadians(1));
    }
}
