package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SlideSubsystem;

@TeleOp
public class SlidePositionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap);
        slideSubsystem.init();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y) slideSubsystem.setState(SlideSubsystem.SlideState.HIGH);
            else if (gamepad1.a) slideSubsystem.setState(SlideSubsystem.SlideState.LOW);

            slideSubsystem.update();
            slideSubsystem.updateTelemetry(telemetry);

            telemetry.update();
        }
    }
}
