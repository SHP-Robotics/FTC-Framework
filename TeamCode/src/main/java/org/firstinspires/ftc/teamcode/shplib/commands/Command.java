package org.firstinspires.ftc.teamcode.shplib.commands;

import java.util.ArrayList;
import java.util.Collections;

public class Command {
    private final Subsystem[] subsystems;
    private final ArrayList<Command> nextCommands = new ArrayList<>();
    private final ArrayList<Command> withCommands = new ArrayList<>();

    public Command(Subsystem... subsystems) {
        this.subsystems = subsystems;
    }

    public Subsystem[] getSubsystems() {
        return subsystems;
    }

    public ArrayList<Command> getNextCommands() {
        return nextCommands;
    }

    public ArrayList<Command> getWithCommands() {
        return withCommands;
    }

    public Command then(Command command) {
        nextCommands.add(command);
        return this;
    }

    public Command then(ArrayList<Command> commands) {
        nextCommands.addAll(commands);
        return this;
    }

    public Command with(Command command) {
        withCommands.add(command);
        return this;
    }

    public Command with(ArrayList<Command> commands) {
        withCommands.addAll(commands);
        return this;
    }

    public void init() {
    }

    public void execute() {
    }

    public void end() {
    }

    public boolean isFinished() {
        return true;
    }

    /**
     * WARNING: May not work as intended with method chaining, use with SequentialCommandGroup instead.
     */
//    public void schedule() {
//        CommandScheduler.getInstance().scheduleCommand(this);
//    }
}
