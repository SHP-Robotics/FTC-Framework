package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;

import org.firstinspires.ftc.teamcode.FourBarSubsystem;

@Disabled
@TeleOp
public class FourBarTest extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadInterface gamepadInterface = new GamepadInterface(gamepad1);
        FourBarSubsystem fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        fourBarSubsystem.setState(FourBarSubsystem.FourBarState.DOWN);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y) {
                fourBarSubsystem.setState(FourBarSubsystem.FourBarState.UP);
            }
            if (gamepad1.a){
                fourBarSubsystem.setState(FourBarSubsystem.FourBarState.DOWN);
            }
            fourBarSubsystem.update();
        }
    }
}
