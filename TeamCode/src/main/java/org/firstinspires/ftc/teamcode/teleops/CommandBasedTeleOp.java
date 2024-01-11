package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;

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

        new Trigger(gamepad1.right_bumper, new RunCommand(()->{
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            claw.nextRight();

            debounce = Clock.now();
        }));
        new Trigger(gamepad1.left_bumper, new RunCommand(()->{
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            claw.nextLeft();

            debounce = Clock.now();
        }));

        new Trigger(gamepad1.x, new RunCommand(()->{ //all downward stuff
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            if(claw.closed() && arm.getState() == ArmSubsystem.State.INTAKE){
                arm.setState(ArmSubsystem.State.DRIVE);
            }
            else if(arm.getState() == ArmSubsystem.State.DRIVE){
                arm.setState(ArmSubsystem.State.OUTTAKE);
            }

            debounce = Clock.now();
        }));
        new Trigger(gamepad1.triangle, new RunCommand(()->{ //all upward stuff
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            if(arm.getState() == ArmSubsystem.State.OUTTAKE){
                claw.close();
                arm.setState(ArmSubsystem.State.DRIVE);
            }
            else if(arm.getState() == ArmSubsystem.State.DRIVE){
                arm.setState(ArmSubsystem.State.INTAKE);
                claw.open();
            }

            debounce = Clock.now();
        }));

        new Trigger(gamepad1.dpad_left, new RunCommand(() -> {
            plane.setState(PlaneSubsystem.State.LAUNCH);
        }));
        new Trigger(gamepad1.circle, new RunCommand(() -> {
            plane.setState(PlaneSubsystem.State.LOAD);
        }));
        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            lift.setState(LiftSubsystem.State.PREPARE);
        }));
        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            lift.setState(LiftSubsystem.State.CLIMB);
        }));

//        new Trigger(gamepad1.a, new RunCommand(() -> {
//            if (!Clock.hasElapsed(debounce, 0.5)) return;
//            if (arm.clawClosed()) {
//                arm.openClaw();
//                if (arm.atHub()) {
//                    arm.setState(ArmSubsystem.State.BOTTOM);
//                }
//            } else {
//                arm.closeClaw();
//                CommandScheduler.getInstance().scheduleCommand(
//                        new WaitCommand(0.5)
//                                .then(new RunCommand(() -> {
//                                    if (arm.atStacks()) arm.setState(ArmSubsystem.State.LOW);
//                                    else arm.setState(ArmSubsystem.State.HUB);
//                                }))
//                );
//            }
//            debounce = Clock.now();
//        }));

//        new Trigger(gamepad1.b, new DropConeCommand(arm));
//
//        new Trigger(gamepad1.x, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.STACK);
//        }));
//
//        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.HIGH);
//        }));
//
//        new Trigger(gamepad1.dpad_right, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.MIDDLE);
//        }));
//
//        new Trigger(gamepad1.dpad_left, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.LOW);
//        }));
//
//        new Trigger(gamepad1.dpad_down, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.BOTTOM);
//        }));
    }
}
