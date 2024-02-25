package org.firstinspires.ftc.teamcode.shplib.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TemplateSubsystem;

public class OpenClawCommand extends Command {
    private final ClawSubsystem claw;
    private double startTime;
//    private double startTime;

    public OpenClawCommand(ClawSubsystem claw) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(claw);

        this.claw = claw;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        startTime = Clock.now();
        claw.closeClaw();
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        claw.openClaw();
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, 0.2);
    }
}