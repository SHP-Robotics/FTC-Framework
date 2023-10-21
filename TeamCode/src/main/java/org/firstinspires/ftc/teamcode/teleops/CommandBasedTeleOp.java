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
public class CommandBasedTeleOp extends BaseRobot {
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
    }

    @Override
    public void start() {
        super.start();

        debounce = Clock.now();
        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {
        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();

        drive.setDriveBias(arm.getDriveBias());

        new Trigger(gamepad1.a, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            if (arm.clawClosed()) {
                arm.openClaw();
                if (arm.atHub()) {
                    arm.setState(ArmSubsystem.State.BOTTOM);
                }
            } else {
                arm.closeClaw();
                CommandScheduler.getInstance().scheduleCommand(
                        new WaitCommand(0.5)
                                .then(new RunCommand(() -> {
                                    if (arm.atStacks()) arm.setState(ArmSubsystem.State.LOW);
                                    else arm.setState(ArmSubsystem.State.HUB);
                                }))
                );
            }
            debounce = Clock.now();
        }));

        new Trigger(gamepad1.b, new DropConeCommand(arm));

        new Trigger(gamepad1.x, new RunCommand(() -> {
            arm.setState(ArmSubsystem.State.STACK);
        }));

        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            arm.setState(ArmSubsystem.State.HIGH);
        }));

        new Trigger(gamepad1.dpad_right, new RunCommand(() -> {
            arm.setState(ArmSubsystem.State.MIDDLE);
        }));

        new Trigger(gamepad1.dpad_left, new RunCommand(() -> {
            arm.setState(ArmSubsystem.State.LOW);
        }));

        new Trigger(gamepad1.dpad_down, new RunCommand(() -> {
            arm.setState(ArmSubsystem.State.BOTTOM);
        }));
    }
}
