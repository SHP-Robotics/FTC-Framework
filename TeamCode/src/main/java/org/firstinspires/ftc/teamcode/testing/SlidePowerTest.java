package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SlideSubsystem;

@TeleOp
public class SlidePowerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap);
        slideSubsystem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideSubsystem.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            slideSubsystem.setPower(-gamepad1.right_stick_y);
            slideSubsystem.updateTelemetry(telemetry);
            telemetry.update();
        }
    }
}
