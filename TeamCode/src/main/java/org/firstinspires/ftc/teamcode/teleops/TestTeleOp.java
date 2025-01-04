package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;

@TeleOp(name = "Test Tele Op")
public class TestTeleOp extends BaseRobot {
    GamepadInterface gamepadInterface;

    @Override
    public void init(){
        super.init();

        gamepadInterface = new GamepadInterface(gamepad1);
    }
    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop(){
        super.loop();
    }
}
