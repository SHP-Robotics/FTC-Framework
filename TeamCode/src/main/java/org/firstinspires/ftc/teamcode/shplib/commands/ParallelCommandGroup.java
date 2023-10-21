package org.firstinspires.ftc.teamcode.shplib.commands;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelCommandGroup extends Command {
    public ParallelCommandGroup(Command... commands) {
        with(new ArrayList<>(Arrays.asList(commands)));
    }
}
