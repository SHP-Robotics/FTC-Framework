package org.firstinspires.ftc.teamcode.commands.used;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
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
        if(horiz.getState() == HorizSubsystem.State.MANUAL){
            horiz.setState(horiz.prevState);
        }
        if(pivot.getState() == PivotSubsystem.State.MANUAL){
            pivot.setState(pivot.prevState);
        }
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
        time = 0.25;
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if(pivot.getState() != PivotSubsystem.State.OUTTAKING
                && horiz.getState() != HorizSubsystem.State.OUTTAKING
                && pivot.getState() == PivotSubsystem.State.DRIVING){ //driving to intake prep
            pivot.setState(PivotSubsystem.State.TRANSITION);
            horiz.setState(HorizSubsystem.State.INTAKING);
        }
        else if(pivot.getState() == PivotSubsystem.State.OUTTAKING){ //outtaking to driving
            pivot.setState(PivotSubsystem.State.DRIVING);
            horiz.setState(HorizSubsystem.State.DRIVING);
        }
        else if(pivot.getState() == PivotSubsystem.State.PREPAREINTAKE){ //intake prep to intaking
//            pivot.setState(PivotSubsystem.State.TRANSITION);
            horiz.setState(HorizSubsystem.State.INTAKING);
        }
        else if(pivot.getState() == PivotSubsystem.State.INTAKING){ //intaking to intakeextended
//            pivot.setState(PivotSubsystem.State.TRANSITION);
            horiz.setState(HorizSubsystem.State.INTAKINGEXTENDED);
        }
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
       //set superStates
        if(pivot.getState() == PivotSubsystem.State.TRANSITION){ //trans to prepare intake
            pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
        }
        else if(pivot.getState() == PivotSubsystem.State.PREPAREINTAKE){ // prepare intake to intake
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
