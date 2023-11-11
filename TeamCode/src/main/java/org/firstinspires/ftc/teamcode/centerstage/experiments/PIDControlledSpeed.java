package org.firstinspires.ftc.teamcode.centerstage.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.Speed;

@TeleOp(name = "PID Controlled Speed")
public class PIDControlledSpeed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, Speed.PID_CONTROLLED_WITH_OVERRIDE);
        mecanumController.setDriveSpeed(0);

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
        }
    }
}
