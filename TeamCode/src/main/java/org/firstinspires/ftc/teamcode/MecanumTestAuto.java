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
        mecanumController.setDriveSpeed(0.7);

        waitForStart();
        mecanumController.moveInches(60, 60, 60, 60, true);
    }
}
