package org.firstinspires.ftc.teamcode.shplib.commands;

public class Trigger {
    public Trigger(boolean condition, Command command) {
        if (condition) CommandScheduler.getInstance().scheduleCommand(command);
    }
}
