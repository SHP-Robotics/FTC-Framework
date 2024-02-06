package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;

public class IncrementUpArmCommand extends Command {
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final ElbowSubsystem elbow;
    private  double startTime;

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
        startTime = Clock.now();

        if(arm.getState() == ArmSubsystem.State.BOTTOM)
            arm.setState(ArmSubsystem.State.EXTENDED);
        else
            arm.incrementState();
//        arm.nextState();
        wrist.setState(WristSubsystem.State.UP);
        elbow.setState(ElbowSubsystem.State.UP);
    }
//    @Override
//    public void end() {
//        pixelServo.setState(PixelServo.State.OUT);
//    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return true;
//        return Clock.hasElapsed(startTime, 1);
    }
}
