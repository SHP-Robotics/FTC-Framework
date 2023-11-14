package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.MecanumController;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);

        waitForStart();

        mecanumController.calibrateIMUAngleOffset();
        telemetry.addData("IMU", mecanumController.getCalibratedIMUAngle());
        telemetry.update();
        mecanumController.moveInches(1, 1, 1, 1, true);
    }
}
