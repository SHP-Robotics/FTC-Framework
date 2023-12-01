package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.3);

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensorForward");

        waitForStart();
        mecanumController.calibrateIMUAngleOffset();

        //while(opModeIsActive()) {
        //    telemetry.addData("red", colorSensor.red());
        //    telemetry.addData("blue", colorSensor.blue());
        //    telemetry.addData("green", colorSensor.green());
        //    telemetry.update();
        //}

        // TODO: calibrate wheel encoder ticks to inches
        // mecanumController.moveToPosition(0, 24, true);
        // mecanumController.moveInches(24, -24, -24, 24, true);
        // mecanumController.moveToPosition(24, 0, true);
        // TODO: calibrate IMU direction in MecanumController class

        telemetry.addData("CIA", mecanumController.getCalibratedIMUAngle());
        telemetry.update();

        mecanumController.rotateToRadian(Math.toRadians(90), Math.toRadians(1));
        mecanumController.rotateToRadian(Math.toRadians(-90), Math.toRadians(1));
    }
}
