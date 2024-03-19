package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumController;

@Disabled
@Autonomous(name = "Mecanum Test Auto")
public class MecanumTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.3);

        waitForStart();
        mecanumController.motors[0].setPower(0.7);
        sleep(1000);
        mecanumController.motors[1].setPower(0.7);
        sleep(1000);
        mecanumController.motors[2].setPower(0.7);
        sleep(1000);
        mecanumController.motors[3].setPower(0.7);
        sleep(1000);
    }
}
