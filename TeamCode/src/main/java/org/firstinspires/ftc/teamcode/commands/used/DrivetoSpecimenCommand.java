package org.firstinspires.ftc.teamcode.commands.used;

import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.used.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.VerticalSubsystem;

public class DrivetoSpecimenCommand extends Command {
    RotateSubsystem rotate;
    ClawSubsystem claw;
    PivotSubsystem pivot;
    HorizSubsystem horiz;
    VerticalSubsystem vertical;
    double trigger, startTime, endTime;

    public DrivetoSpecimenCommand(RotateSubsystem rotate, ClawSubsystem claw, PivotSubsystem pivot, HorizSubsystem horiz, VerticalSubsystem vertical) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(rotate, claw, pivot, horiz);
        this.rotate = rotate;
        this.claw = claw;
        this.pivot = pivot;
        this.horiz = horiz;
        this.vertical = vertical;
        endTime = 0.5;
    }


    // Called once when the command is initially schedule

    public void init() {
        super.init();
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        horiz.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.OUTTAKE);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        //TODO raise slides
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        rotate.setState(RotateSubsystem.State.DROPOFF);

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
