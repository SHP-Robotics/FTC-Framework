package org.firstinspires.ftc.teamcode.shplib.commands;

import java.util.ArrayList;
import java.util.Arrays;

public class SequentialCommandGroup extends Command {

    public SequentialCommandGroup(Command... commands) {
        ArrayList<Command> nextCommands = new ArrayList<>(Arrays.asList(commands));
        unpack(nextCommands);
    }

    private void unpack(ArrayList<Command> commands) {
        for (Command command : commands) {
            if (command instanceof SequentialCommandGroup) {
                unpack(command.getNextCommands());
            } else then(command);
        }
    }
}
