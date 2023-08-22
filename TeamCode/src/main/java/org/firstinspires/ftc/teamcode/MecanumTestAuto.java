package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.generals.MecanumController;
import org.firstinspires.ftc.teamcode.generals.RuntimeType;

@Autonomous(name = "Mecanum Test Auto")
public class MecanumTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, RuntimeType.AUTONOMOUS);
        mecanumController.setDriveSpeed(0.3);

        waitForStart();
        mecanumController.frontLeft.setPower(0.7);
        sleep(1000);
        mecanumController.frontRight.setPower(0.7);
        sleep(1000);
        mecanumController.backLeft.setPower(0.7);
        sleep(1000);
        mecanumController.backRight.setPower(0.7);
        sleep(1000);
    }
}
