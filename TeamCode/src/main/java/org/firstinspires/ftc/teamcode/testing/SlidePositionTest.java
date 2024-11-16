package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;

import org.firstinspires.ftc.teamcode.SlideSubsystem;

@Disabled
@TeleOp
public class SlidePositionTest extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadInterface gamepadInterface = new GamepadInterface(gamepad1);
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap);
        slideSubsystem.init();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y) {
                if (slideSubsystem.getState() == SlideSubsystem.SlideState.INTAKE)
                    slideSubsystem.setState(SlideSubsystem.SlideState.LOW);
                else
                    slideSubsystem.setState(SlideSubsystem.SlideState.HIGH);
            }
            if (gamepad1.a){
                if (slideSubsystem.getState() == SlideSubsystem.SlideState.HIGH)
                    slideSubsystem.setState(SlideSubsystem.SlideState.LOW);
                else
                    slideSubsystem.setState(SlideSubsystem.SlideState.INTAKE);
            }
            slideSubsystem.update();
            slideSubsystem.updateTelemetry(telemetry);
            telemetry.update();
        }
    }
}
