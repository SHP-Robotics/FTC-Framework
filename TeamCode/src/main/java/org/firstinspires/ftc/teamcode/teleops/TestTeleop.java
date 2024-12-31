package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class TestTeleop extends BaseRobot {
    private double debounce;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.newMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
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

        //Vertical Slides
        new Trigger(gamepad1.touchpad_finger_1, new RunCommand(() -> {
            vertical.incrementSlide();

        }));
        new Trigger(gamepad1.left_bumper, new RunCommand(() -> {
            vertical.decrementSlide();
        }));

        //Spinning Intake
        new Trigger(gamepad1.right_trigger>0.1, new RunCommand(() -> {
//            intake.setPower(-gamepad1.right_trigger);
        }));
        new Trigger(gamepad1.left_trigger>0.1, new RunCommand(() -> {
//            intake.setPower(gamepad1.left_trigger);
        }));

        //Horizontal + Pivot

//        new Trigger(gamepad1.right_bumper, new HorizontalDownOutCommand(horizontal, pivot));
//        new Trigger(gamepad1.left_bumper, new HorizontalUpInCommand(horizontal, pivot));


        //Claw
        new Trigger(gamepad2.triangle, new RunCommand(() -> {
//            if(debounce > 0.5) {
//                debounce = Clock.now();
//                claw.close();
//            }
        }));
        new Trigger(gamepad2.cross, new RunCommand(()->{
//            claw.open();
        }));

        //reset Slide Zero Pos
        new Trigger(gamepad2.square, new RunCommand(()->{
            vertical.resetZeroPosition();
        }));

        //resetIMU
        new Trigger(gamepad1.square, new RunCommand(()->{
            drive.resetIMUAngle();
        }));

    }


}
