package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.used.HorizontalDownOutCommand;
import org.firstinspires.ftc.teamcode.commands.used.HorizontalUpInCommand;
import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.used.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.PivotSubsystem;

@TeleOp
public class TestTeleop extends BaseRobot {
    private double debounce, timeElapsed;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );

    }
    @Override
    public void start(){
        super.start();
        debounce = Clock.now();
        timeElapsed = Clock.now();
    }

    @Override
    public void loop(){
        super.loop();

        //Vertical Slides
        new Trigger(gamepad2.right_bumper, new RunCommand(() -> {
            vertical.incrementSlide();

        }));
        new Trigger(gamepad2.left_bumper, new RunCommand(() -> {
            vertical.decrementSlide();
        }));

        //Spinning Intake
        new Trigger(gamepad1.right_trigger>0.1, new RunCommand(() -> {
            intake.intaking(-gamepad1.right_trigger);
        }));
        new Trigger(gamepad1.left_trigger>0.1, new RunCommand(() -> {
            intake.intaking(gamepad1.left_trigger);
        }));

        //Horizontal + Pivot
        new Trigger(gamepad1.right_bumper, new HorizontalDownOutCommand(horizontal, pivot));
        new Trigger(gamepad1.left_bumper, new HorizontalUpInCommand(horizontal, pivot));


        //Claw
        new Trigger(gamepad1.triangle, new RunCommand(() -> {
            if(timeElapsed > 0.5) {
                timeElapsed = Clock.now();
                claw.changeClaw();
            }
        }));

        debounce = Clock.now();

    }


}
