package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;

public class LowerArmCommand extends Command {
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final ElbowSubsystem elbow;
    private  double startTime;

    public LowerArmCommand(ArmSubsystem arm, WristSubsystem wrist, ElbowSubsystem elbow) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(arm, wrist, elbow);

        this.arm = arm;
        this.wrist = wrist;
        this.elbow = elbow;

    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        startTime = Clock.now();
        arm.setState(ArmSubsystem.State.BOTTOM);
        wrist.setState(WristSubsystem.State.HALFWAY);
        elbow.setState(ElbowSubsystem.State.DOWN);


    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once

    @Override
    public void end() {
        wrist.setState(WristSubsystem.State.DOWN);
    }

    @Override
    public boolean isFinished() {
        return arm.getSlidePosition() < Constants.Arm.kSlideTolerance;
    }
}
