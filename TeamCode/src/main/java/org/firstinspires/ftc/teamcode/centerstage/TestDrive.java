package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.*;

@TeleOp(name = "Test Drive (Adaptive Speeds)")
public class TestDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, Speed.PID_CONTROLLED_WITH_OVERRIDE);
        mecanumController.setDriveSpeed(0.7);

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
        }
    }
}
