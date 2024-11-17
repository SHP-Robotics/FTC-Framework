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
        DRIVING(0.74, 1), //1 is all in
        INTAKING(0.17,1),
        INTAKINGEXTENDED(0.17,0.3), // 0.14 rail max out, 0.3 slide max out
        MANUAL(1,1);

        final double railPos;
        final double slidePos;

        State(double railPos, double slidePos) {
            this.railPos = railPos;
            this.slidePos = slidePos;
        }
    }

    private State state;
    public State prevState;
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

        setState(State.DRIVING);
        prevState = state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void nextState(){
        if(state == State.DRIVING)
            state = State.INTAKINGEXTENDED;
        else if (state == State.INTAKINGEXTENDED)
            state = State.INTAKING;
        else
            state = State.DRIVING;
    }

    public void incrementState(){
        if(state == State.DRIVING)
            state = State.INTAKING;
        else if (state == State.INTAKING)
            state = State.INTAKINGEXTENDED;
        else
            state = State.INTAKINGEXTENDED;
    }
    public void decrementState(){
        if(state == State.INTAKINGEXTENDED)
            state = State.INTAKING;
        else if (state == State.INTAKING)
            state = State.DRIVING;
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
        if (this.state == State.OUTTAKING
                || this.state == State.DRIVING
                || this.state == State.INTAKINGEXTENDED
                || this.state == State.INTAKING) {
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

        telemetry.addData("HorizState: ", state);
        telemetry.addData("Left Horiz Slide Position: ", lHoriz.getPosition());
        telemetry.addData("Right Horiz Slide Position: ", rHoriz.getPosition());
        telemetry.addData("rail: ", rail.getPosition());

//        telemetry.addData("Left Slide Velocity: ", leftSlide.getVelocity());
//        telemetry.addData("Right Slide Velocity: ", rightSlide.getVelocity());
    }
}
