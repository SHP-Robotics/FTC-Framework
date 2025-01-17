package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.WristSubsystem.WristState.DOWN;
import static org.firstinspires.ftc.teamcode.WristSubsystem.WristState.UP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.WristSubsystem;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadInterface gamepadInterface = new GamepadInterface(gamepad1);
        WristSubsystem wristSubsystem = new WristSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            gamepadInterface.update();

            if (gamepadInterface.isKeyDown(GamepadKey.RIGHT_BUMPER)) {
                if (wristSubsystem.getState() == DOWN)
                    wristSubsystem.setState(UP);
                else
                    wristSubsystem.setState(DOWN);
            }

            wristSubsystem.update();

            telemetry.addData("state", wristSubsystem.getState());
            telemetry.update();
        }
    }
}
