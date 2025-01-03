package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class ATuningTeleop extends BaseRobot {

    private double debounce;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );

    }
    @Override
    public void start(){
        super.start();
        debounce = Clock.now();
    }

    @Override
    public void loop(){
        super.loop();

        new Trigger(gamepad1.dpad_up, new RunCommand(()-> {
            pivot.incrementElbowUp();
        }));
        new Trigger(gamepad1.dpad_down, new RunCommand(()-> {
            pivot.decrementElbowDown();
        }));

        new Trigger(gamepad1.dpad_right, new RunCommand(() ->{
            pivot.incrementWristUp();
        }));
        new Trigger(gamepad1.dpad_left, new RunCommand(() ->{
            pivot.decrementWristDown();
        }));


        new Trigger(gamepad1.right_trigger > 0.1, new RunCommand(() -> {
            vertical.incrementSlide();
        }));
        new Trigger(gamepad1.left_trigger > 0.1, new RunCommand(() -> {
            vertical.decrementSlide();
        }));

        new Trigger(gamepad1.circle, new RunCommand(() -> {
            horizontal.incrementHorizSlide();
        }));
        new Trigger(gamepad1.square, new RunCommand(() -> {
            horizontal.decrementHorizSlide();
        }));
        new Trigger(gamepad1.triangle, new RunCommand(() -> {
            horizontal.incrementRail();
        }));
        new Trigger(gamepad1.cross, new RunCommand(() -> {
            horizontal.decrementRail();
        }));

        new Trigger(gamepad2.dpad_left, new RunCommand(() -> {
            rotate.rotateCCW();
        }));
        new Trigger(gamepad2.dpad_right, new RunCommand(() -> {
            rotate.rotateCW();
        }));

        new Trigger(gamepad2.circle, new RunCommand(() -> {
            claw.increment();
        }));new Trigger(gamepad2.square, new RunCommand(() -> {
            claw.decrement();
        }));



        debounce = Clock.now();

    }



}
