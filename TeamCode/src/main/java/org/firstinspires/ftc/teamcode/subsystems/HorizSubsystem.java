package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kLeftHorizSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRailName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRightHorizSlideName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class HorizSubsystem extends Subsystem {
    private final CachingServo lHoriz;
    private final CachingServo rHoriz;
    private final CachingServo rail;

    public enum State {
        INTAKEWALL(0, 0),
        AUTOINTAKE(0.5, 0),
        DRIVING(0, 0), //1, 0.725 is all in
        INTAKING(1,0.5),
        INTAKINGEXTENDED(1,1), // 0.55 rail max out, 0 slide max out
        MANUAL(0.7,0);

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
        manualHorizPos = 0.0;
        manualRailPos = 0.0;

        lHoriz = new CachingServo((Servo) hardwareMap.get(kLeftHorizSlideName));
        lHoriz.scaleRange(0.325, 0.9);
        lHoriz.setDirection(Servo.Direction.REVERSE);

        rHoriz = new CachingServo((Servo) hardwareMap.get(kRightHorizSlideName));
        lHoriz.scaleRange(0.325, 0.9);
        rHoriz.setDirection(Servo.Direction.FORWARD);

        rail = new CachingServo((Servo) hardwareMap.get(kRailName));
        rail.setDirection(Servo.Direction.FORWARD);
        rail.scaleRange(0, 0.45); //0 in, 0.45 out

        setState(State.DRIVING);
        prevState = state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void incrementHorizSlide(){
        if(manualHorizPos < 1.0) {
            state = State.MANUAL;
            manualHorizPos = lHoriz.getPosition() + 0.01;
        }
    }

    public void decrementHorizSlide(){
        if(manualHorizPos > 0.0) {
            state = State.MANUAL;
            manualHorizPos = lHoriz.getPosition() - 0.01;
        }
    }
    public void incrementRail(){
        if(manualRailPos < 1.0) {
            state = State.MANUAL;
            manualRailPos = rail.getPosition() + 0.01;
        }
    }

    public void decrementRail(){
        if(manualRailPos > 0.0) {
            state = State.MANUAL;
            manualRailPos = rail.getPosition() - 0.01;
        }
    }

    public void setPos(double pos){
        state = State.MANUAL;
        manualHorizPos = 0.65 *(1-pos);
        manualRailPos = rail.getPosition();

    }

    public void setTriggerPos(double trigger){
        state = State.MANUAL;
        if(trigger < 0.15){
            manualHorizPos = 0.0;
            manualRailPos = trigger * 2 + 0.7;
        }
        else {
            manualHorizPos = trigger;
            manualRailPos = 1.0;
        }
    }


    private void setHorizPosition(double position){
        lHoriz.setPosition(position);
        rHoriz.setPosition(position);
    }

    private void processState(State state) {
        if (this.state != State.MANUAL) {
            setHorizPosition(this.state.slidePos);
            rail.setPosition(this.state.railPos);
        }
        else {
            setHorizPosition(manualHorizPos);
            rail.setPosition(manualRailPos);
        }
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState(state);

        telemetry.addData("Horiz State: ", state);
        telemetry.addData("Left Horiz Position: ", lHoriz.getPosition());
        telemetry.addData("Right Horiz Position: ", rHoriz.getPosition());
        telemetry.addData("Rail Pos: ", rail.getPosition());

    }
}
