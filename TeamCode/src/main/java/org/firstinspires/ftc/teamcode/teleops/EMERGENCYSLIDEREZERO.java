package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;

@TeleOp
public class EMERGENCYSLIDEREZERO extends BaseRobot {
    private double driveBias;
    private boolean holdingRightBumper, LBTrigger, LTTrigger, crossTrigger;

    GamepadInterface gamepadInterface2;

    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-driveBias*gamepad1.left_stick_y, driveBias*gamepad1.left_stick_x, driveBias*gamepad1.right_stick_x)
                )
        );
        holdingRightBumper = false;
        LBTrigger = true;
        LTTrigger = true;
        crossTrigger = true;

        gamepadInterface2 = new GamepadInterface(gamepad2);
    }
    @Override
    public void start(){
        super.start();
        driveBias = vertical.getDriveBias();

        //pivot.setState(PivotSubsystem.State.DRIVING);
    }

    @Override
    public void loop(){
        super.loop();
        driveBias = vertical.getDriveBias();
        gamepadInterface2.update();
        drive.update(gamepad2);
//        drive.drive.setZeroPowerBehavior(gamepad2.cross ? BRAKE : FLOAT);
        // use gamepad2.right_bumper as speed boost

        //fix slides
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_UP), new RunCommand(() -> {
            vertical.incrementSlide();
        }));
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_DOWN), new RunCommand(()->{
            vertical.emergencyDecrementSlide();
        }));

    }
}
