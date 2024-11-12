package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kLeftHorizSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRailName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRightHorizSlideName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class HorizSubsystem extends Subsystem {
    private final Servo lHoriz;
    private final Servo rHoriz;
    private final Servo rail;

    private int slidePos;

    public enum State {
        OUTTAKING(1, 1),
        ALLIN(0.74, 1), //1 is all in
        HALFOUT(0.17,1),
        ALLOUT(0.17,0.3), // 0.14 rail max out, 0.3 slide max out
        MANUAL(1,1);

        final double railPos;
        final double slidePos;

        State(double railPos, double slidePos) {
            this.railPos = railPos;
            this.slidePos = slidePos;
        }
    }

    private State state;
    private double manualHorizPos, manualRailPos;

    public HorizSubsystem(HardwareMap hardwareMap) {
        slidePos = 0;
        manualHorizPos = 0.0;
        manualRailPos = 0.0;

        lHoriz = (Servo) hardwareMap.get(kLeftHorizSlideName);
        lHoriz.setDirection(Servo.Direction.REVERSE);

        rHoriz = (Servo) hardwareMap.get(kRightHorizSlideName);
        rHoriz.setDirection(Servo.Direction.FORWARD);

        rail = (Servo) hardwareMap.get(kRailName);
        rail.setDirection(Servo.Direction.REVERSE);

        setState(State.ALLIN);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void nextState(){
        if(state == State.ALLIN)
            state = State.ALLOUT;
        else if (state == State.ALLOUT)
            state = State.HALFOUT;
        else
            state = State.ALLIN;
    }

    public void incrementState(){
        if(state == State.ALLIN)
            state = State.HALFOUT;
        else if (state == State.HALFOUT)
            state = State.ALLOUT;
        else
            state = State.ALLOUT;
    }
    public void decrementState(){
        if(state == State.ALLOUT)
            state = State.HALFOUT;
        else if (state == State.HALFOUT)
            state = State.ALLIN;
    }

    public void incrementHorizSlide(){
        if(manualHorizPos < 1.0) {
            state = State.MANUAL;
            manualHorizPos += 0.01;
        }
    }

    public void decrementHorizSlide(){
        if(manualHorizPos > 0.0) {
            state = State.MANUAL;
            manualHorizPos -= 0.01;
        }
    }
    public void incrementRail(){
        if(manualRailPos < 1.0) {
            state = State.MANUAL;
            manualRailPos += 0.01;
        }
    }

    public void decrementRail(){
        if(manualRailPos > 0.0) {
            state = State.MANUAL;
            manualRailPos -= 0.01;
        }
    }

    private void setHorizPosition(double position){
        lHoriz.setPosition(position);
        rHoriz.setPosition(1.0-position);
    }

    private void processState(State state) {
        if (this.state == State.ALLIN || this.state == State.ALLOUT
                || this.state == State.HALFOUT) {
            setHorizPosition(this.state.slidePos);
            rail.setPosition(this.state.railPos);
            manualHorizPos = state.slidePos;
            manualRailPos = state.railPos;
            return;
        }
        else if(this.state == State.MANUAL){
            setHorizPosition(manualHorizPos);
            rail.setPosition(manualRailPos);
        }

//        this.setPosition(this.state.position);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState(state);

        telemetry.addData("State: ", state);
        telemetry.addData("Left Horiz Slide Position: ", lHoriz.getPosition());
        telemetry.addData("Right Horiz Slide Position: ", rHoriz.getPosition());
        telemetry.addData("rail: ", rail.getPosition());

//        telemetry.addData("Left Slide Velocity: ", leftSlide.getVelocity());
//        telemetry.addData("Right Slide Velocity: ", rightSlide.getVelocity());
    }
}
