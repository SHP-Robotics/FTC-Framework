package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.MecanumController;

// comparing SHP lib to debug
// approximately equal
@Disabled
@TeleOp
public class DriveBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        waitForStart();
        mecanumController.calibrateIMUAngleOffset();
        while (opModeIsActive() && !isStopRequested()) mecanumController.fieldOrientedDrive(gamepad1);
    }
}
