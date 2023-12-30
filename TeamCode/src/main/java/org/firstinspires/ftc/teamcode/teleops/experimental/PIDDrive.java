package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;

@Disabled
@TeleOp(group = "experimental")
public class PIDDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.PID_CONTROLLED_WITH_OVERRIDE)
                .setOverrideOneSpeed(0.3)
                .build();
        MecanumController mecanumController = new MecanumController(hardwareMap, speedController);

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
            telemetry.addData("current speed", mecanumController.getDriveSpeed());
            telemetry.update();
        }
    }
}
