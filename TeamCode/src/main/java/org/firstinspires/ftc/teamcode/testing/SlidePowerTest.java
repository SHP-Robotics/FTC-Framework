package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SlideSubsystem;

@Disabled
@TeleOp
public class SlidePowerTest extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap);
        slideSubsystem.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            slideSubsystem.setPower(-gamepad1.right_stick_y);
        }
    }
}
