package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generals.*;

@TeleOp(name = "Mecanum Wheel Drive")
public class MecanumWheelDrive extends LinearOpMode {
    MecanumController mecanumController;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumController = new MecanumController(hardwareMap, RuntimeType.DRIVER_CONTROLLED_TELEOP);

        waitForStart();
        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
        }
    }
}
