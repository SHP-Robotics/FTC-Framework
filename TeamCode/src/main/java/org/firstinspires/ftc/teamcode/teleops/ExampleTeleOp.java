package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DropConeCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp
public class ExampleTeleOp extends BaseRobot {
    private double debounce;

    @Override
    public void init() {
        super.init();

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x)
                )
        );
        //Add any additional code here:


    }

    @Override
    public void start() {
        super.start();

        debounce = Clock.now();
        // Add anything that needs to be run a single time when the OpMode starts:


    }

    @Override
    public void loop() {
        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();

//        drive.setDriveBias(arm.getDriveBias());

//        new Trigger(gamepad1.b, new DropConeCommand());

//
        new Trigger(gamepad1.x, new RunCommand(() -> {
            arm.openclaw();
        }));

//        new Trigger(gamepad1.dpad_down, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.BOTTOM);
//        }));
    }
}
