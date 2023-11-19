package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.MecanumController;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);

        telemetry.addData("speed", mecanumController.getDriveSpeed());
        telemetry.update();

        waitForStart();

        mecanumController.leftFront.setPower(1);
        sleep(1000);
        mecanumController.rightFront.setPower(1);
        sleep(1000);
        mecanumController.leftRear.setPower(1);
        sleep(1000);
        mecanumController.rightRear.setPower(1);
        sleep(1000);
    }
}
