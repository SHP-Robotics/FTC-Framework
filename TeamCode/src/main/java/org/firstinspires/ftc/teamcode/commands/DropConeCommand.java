package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TemplateSubsystem;

public class DropConeCommand extends Command {
    private final ArmSubsystem arm;
    private double startTime;
//    private double startTime;

    public DropConeCommand(ArmSubsystem arm) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(arm);

        this.arm = arm;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        startTime = Clock.now();
        arm.setState(ArmSubsystem.State.BOTTOM);
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        arm.openClaw();
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, 0.2);
    }
}
