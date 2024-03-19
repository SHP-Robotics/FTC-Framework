package org.firstinspires.ftc.teamcode.shplib.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Subsystem {
    private RunCommand defaultCommand;

    public Subsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public RunCommand getDefaultCommand() {
        return defaultCommand;
    }

    public void setDefaultCommand(RunCommand defaultCommand) {
        this.defaultCommand = defaultCommand;
    }

    public void periodic(Telemetry telemetry) throws InterruptedException { }
}
