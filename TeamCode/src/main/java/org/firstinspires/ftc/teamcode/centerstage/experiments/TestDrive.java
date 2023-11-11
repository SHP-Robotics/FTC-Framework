package org.firstinspires.ftc.teamcode.centerstage.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.debug.*;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

@TeleOp(name = "Test Drive (Adaptive Speeds)")
public class TestDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, Speed.PID_CONTROLLED_WITH_OVERRIDE);

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
        }
    }
}
