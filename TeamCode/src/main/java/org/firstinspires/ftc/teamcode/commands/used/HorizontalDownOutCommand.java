package org.firstinspires.ftc.teamcode.commands.used;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.used.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.PivotSubsystem;

public class HorizontalDownOutCommand extends Command {
    private final HorizSubsystem horiz;
    private final PivotSubsystem pivot;
    private double startTime;
    private double endTime;
    double time;

    public HorizontalDownOutCommand(HorizSubsystem horiz, PivotSubsystem pivot) {
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
        if(pivot.getState() == PivotSubsystem.State.OUTTAKING) //if outtaking
            pivot.setState(PivotSubsystem.State.DRIVING);
        else if(horiz.getState() == HorizSubsystem.State.ALLIN){ //if transition is needed
            horiz.setState(HorizSubsystem.State.HALFOUT);
        }
        else if(horiz.getState() == HorizSubsystem.State.HALFOUT){
            pivot.setState(PivotSubsystem.State.TRANSITION);
        }


    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        if(pivot.getState() == PivotSubsystem.State.DRIVING){
            horiz.setState(HorizSubsystem.State.ALLIN);
        }
        else if(horiz.getState() == HorizSubsystem.State.ALLIN) { //if driving to half out
             pivot.setState(PivotSubsystem.State.INTAKING);
         }
        else if(horiz.getState() == HorizSubsystem.State.HALFOUT) //if half out to all out
            horiz.setState(HorizSubsystem.State.ALLOUT);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, time);
    }
}
