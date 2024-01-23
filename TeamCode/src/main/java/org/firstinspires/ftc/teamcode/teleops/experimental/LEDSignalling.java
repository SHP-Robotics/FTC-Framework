package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class LEDSignalling extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() throws InterruptedException {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriver.setPattern(pattern);
    }
}
