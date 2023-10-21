package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.TemplateSubsystem;

public class TemplateCommand extends Command {
    private final TemplateSubsystem template;

    public TemplateCommand(TemplateSubsystem template) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(template);

        this.template = template;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {

    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {

    }

    // Called once after isFinished() returns true
    @Override
    public void end() {

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return true;
    }
}
