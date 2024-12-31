package org.firstinspires.ftc.teamcode.commands.used;

import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.used.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.RotateSubsystem;

public class WalltoDriveCommand extends Command {
    RotateSubsystem rotate;
    ClawSubsystem claw;
    PivotSubsystem pivot;
    HorizSubsystem horiz;
    private double startTime;
    private double endTime;

    public WalltoDriveCommand(RotateSubsystem rotate, ClawSubsystem claw, PivotSubsystem pivot, HorizSubsystem horiz) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(rotate, claw, pivot, horiz);
        this.rotate = rotate;
        this.claw = claw;
        this.pivot = pivot;
        this.horiz = horiz;
        endTime = 0.5;
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        claw.close();
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        pivot.setState(PivotSubsystem.State.PICKUP2);
        rotate.setState(RotateSubsystem.State.NEUTRAL);

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
