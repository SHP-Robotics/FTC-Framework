package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.RuntimeType;

@Autonomous(name = "Mecanum Test Auto")
public class MecanumTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, RuntimeType.AUTONOMOUS);
        mecanumController.setDriveSpeed(0.3);

        waitForStart();
        mecanumController.leftFront.setPower(0.7);
        sleep(1000);
        mecanumController.rightFront.setPower(0.7);
        sleep(1000);
        mecanumController.leftRear.setPower(0.7);
        sleep(1000);
        mecanumController.rightRear.setPower(0.7);
        sleep(1000);
    }
}
