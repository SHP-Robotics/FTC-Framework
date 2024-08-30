package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Cycle Blinkin LED")
public class CycleBlinkinLED extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode()
    {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        telemetry.addData("Pattern: ", pattern.toString());
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.b) {
                pattern = pattern.next();
                blinkinLedDriver.setPattern(pattern);
                telemetry.addData("Pattern: ", pattern.toString());
                telemetry.update();
            } else if (gamepad1.x) {
                pattern = pattern.previous();
                blinkinLedDriver.setPattern(pattern);
                telemetry.addData("Pattern: ", pattern.toString());
                telemetry.update();
            }
        }
    }
}
