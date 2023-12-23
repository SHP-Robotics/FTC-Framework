package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;

@Disabled
@TeleOp(name = "PID Controlled Speed")
public class PIDControlledSpeed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.PID_CONTROLLED_WITH_OVERRIDE)
                .build();

        MecanumController mecanumController = new MecanumController(hardwareMap, speedController);
        mecanumController.setDriveSpeed(0);

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
        }
    }
}
