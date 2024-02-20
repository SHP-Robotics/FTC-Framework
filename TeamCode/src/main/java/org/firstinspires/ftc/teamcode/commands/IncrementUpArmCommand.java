package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class IncrementUpArmCommand extends Command {
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final ElbowSubsystem elbow;

    public IncrementUpArmCommand(ArmSubsystem arm, WristSubsystem wrist, ElbowSubsystem elbow) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(arm, wrist, elbow);

        this.arm = arm;
        this.wrist = wrist;
        this.elbow = elbow;

    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        if(arm.getState() == ArmSubsystem.State.BOTTOM) {
            arm.setState(ArmSubsystem.State.EXTENDED);
            wrist.setState(WristSubsystem.State.UP);
            elbow.setState(ElbowSubsystem.State.UP);
        } else {
            arm.incrementState();
        }

        CommandScheduler.getInstance().removeInstances(LowerArmCommand.class);
        // not deemed necessary atm, no driving bugs found without this line of code
//        CommandScheduler.getInstance().removeInstances(DecrementDownArmCommand.class);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
