package org.firstinspires.ftc.teamcode.commands.used;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.used.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.PivotSubsystem;

public class HorizontalUpInCommand extends Command {
    private final HorizSubsystem horiz;
    private final PivotSubsystem pivot;
    private double startTime;
    private double endTime;
    double time;

    public HorizontalUpInCommand(HorizSubsystem horiz, PivotSubsystem pivot) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(horiz, pivot);
        this.horiz = horiz;
        this.pivot = pivot;
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
        time = 0.25;
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if(horiz.getState() == HorizSubsystem.State.HALFOUT){
            pivot.setState(PivotSubsystem.State.TRANSITION);
        }
        else if(horiz.getState() == HorizSubsystem.State.ALLOUT){
            pivot.setState(PivotSubsystem.State.TRANSITION);
        }
        else if(horiz.getState() == HorizSubsystem.State.ALLIN){
            pivot.setState(PivotSubsystem.State.OUTTAKING);
        }


    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        if(pivot.getState() == PivotSubsystem.State.DRIVING){
            horiz.setState(HorizSubsystem.State.OUTTAKING);
        }
        else if(horiz.getState() == HorizSubsystem.State.HALFOUT) {
             horiz.setState(HorizSubsystem.State.ALLIN);
             pivot.setState(PivotSubsystem.State.DRIVING);
        }
        else if(horiz.getState() == HorizSubsystem.State.ALLOUT) {
            horiz.setState(HorizSubsystem.State.HALFOUT);
            pivot.setState(PivotSubsystem.State.INTAKING);
        }

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, time);
    }
}
