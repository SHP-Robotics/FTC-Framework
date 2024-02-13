package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class OpenStageDoorCommand extends Command {
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final ElbowSubsystem elbow;

    public OpenStageDoorCommand(ArmSubsystem arm, WristSubsystem wrist, ElbowSubsystem elbow) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(arm, wrist, elbow);

        this.arm = arm;
        this.wrist = wrist;
        this.elbow = elbow;

    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        if(arm.getState() == ArmSubsystem.State.BOTTOM)
            wrist.setState(WristSubsystem.State.STAGE_DOOR);
            elbow.setState(ElbowSubsystem.State.STAGE_DOOR);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
