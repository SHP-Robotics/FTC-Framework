package org.firstinspires.ftc.teamcode.shplib.commands;

import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

public class WaitCommand extends Command {
    private final double seconds;
    private double startTime;

    public WaitCommand(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void init() {
        startTime = Clock.now();
    }

    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, seconds);
    }
}
