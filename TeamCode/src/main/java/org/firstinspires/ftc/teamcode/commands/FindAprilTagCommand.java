package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class FindAprilTagCommand extends Command {
    private final VisionSubsystem vision;
    private double startTime;

    public FindAprilTagCommand(VisionSubsystem vision) {
        // Pass through any subsystems that are uninterruptible
        super(vision);

        this.vision = vision;
    }

    @Override
    public void init() {
        this.startTime = Clock.now();
        vision.setState(VisionSubsystem.State.ENABLED);
    }

    @Override
    public boolean isFinished() {
        return vision.detectedTags() || Clock.hasElapsed(startTime, 10);
    }

    @Override
    public void end() {
        vision.setState(VisionSubsystem.State.DISABLED);
    }
}
