package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.kWristName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.klElbowName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.krElbowName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class PivotSubsystem extends Subsystem {
    private final Servo wrist;
    private final Servo lElbow;
    private final Servo rElbow;

    public enum State {
//        TRANSITION(0.6,0.3), //Rename to some TRANSITION STATE
        DRIVING(0.6, 0.2), //0 is down
        PREPAREDRIVING(0.2, 0.2), //0 is down

        PREPAREINTAKE(0.6, 0.06),
        INTAKE(0.63,0), //  0.63 wrist is level with floor
        PICKUP(0.925,0.2),
        PICKUP2(0.925,0.4),
        PICKUP3(0.6,0.3),

        OUTTAKE(0.05,0.6),
        MANUAL(0,0);

        final double wristPos;
        final double elbowPos;

        State(double wristPos, double elbowPos) {
            this.wristPos = wristPos;
            this.elbowPos = elbowPos;
        }
    }

    private State state;
    public State prevState;
    private double manualWristPos, manualElbowPos;

    public PivotSubsystem(HardwareMap hardwareMap){
        wrist = (Servo) hardwareMap.get(kWristName);
        lElbow = (Servo) hardwareMap.get(klElbowName);
        lElbow.setDirection(Servo.Direction.REVERSE);
        rElbow = (Servo) hardwareMap.get(krElbowName);

        setState(State.DRIVING);
        prevState = state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void setWristPos(double pos){
        wrist.setPosition(pos);
    }
    public void setElbowPos(double pos){
        lElbow.setPosition(pos);
        rElbow.setPosition(pos);
    }
    public void incrementElbowUp(){
        if(lElbow.getPosition() < 0.83) {
            state = State.MANUAL;
            manualElbowPos = lElbow.getPosition() + 0.01;
        }
    }
    public void decrementElbowDown(){
        if(lElbow.getPosition() > 0.0) {
            state = State.MANUAL;
            manualElbowPos = lElbow.getPosition() - 0.01;
        }
    }
    public void incrementWristUp(){
        if(wrist.getPosition() < 1.0) {
            state = State.MANUAL;
            manualWristPos = wrist.getPosition() + 0.01;
        }
    }
    public void decrementWristDown(){
        if(wrist.getPosition() > 0.0) {
            state = State.MANUAL;
            manualWristPos = wrist.getPosition() - 0.01;

        }
    }

    private void processState(State state) {
        if (this.state != State.MANUAL) {
            setElbowPos(this.state.elbowPos);
            setWristPos(this.state.wristPos);
        }
        else {
            setElbowPos(manualElbowPos);
            setWristPos(manualWristPos);
        }
    }
    @Override
    public void periodic(Telemetry telemetry) {
        processState(state);

        telemetry.addData("Pivot State: ", state);
        telemetry.addData("Left Elbow Position: ", lElbow.getPosition());
        telemetry.addData("Right Elbow Position: ", lElbow.getPosition());
        telemetry.addData("Wrist: ", wrist.getPosition());

//        telemetry.addData("Left Slide Velocity: ", leftSlide.getVelocity());
//        telemetry.addData("Right Slide Velocity: ", rightSlide.getVelocity());
    }


}
