package org.firstinspires.ftc.teamcode.shplib.commands;

public class RunCommand extends Command {
    Runnable toRun;

    public RunCommand(Runnable toRun, Subsystem... subsystems) {
        super(subsystems);
        this.toRun = toRun;
    }

    @Override
    public void execute() {
        toRun.run();
    }
}
