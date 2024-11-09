package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.kWristName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.klElbowName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.krElbowName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class PivotSubsystem extends Subsystem {
    private final IntakeSubsystem intake;
    private final Servo wrist;
    private final Servo lElbow;
    private final Servo rElbow;

    public enum State {
        DRIVING(0, 0),
        INTAKING(1,0),
        OUTTAKING(1,1),
        MANUAL(0,0);

        final double wristPos;
        final double elbowPos;

        State(double wristPos, double elbowPos) {
            this.wristPos = wristPos;
            this.elbowPos = elbowPos;
        }
    }

    private State state;
    private double manualWristPos, manualElbowPos;

    public PivotSubsystem(HardwareMap hardwareMap){
        intake = new IntakeSubsystem(hardwareMap);
        wrist = (Servo) hardwareMap.get(kWristName);
        lElbow = (Servo) hardwareMap.get(klElbowName);
        lElbow.setDirection(Servo.Direction.REVERSE);
        rElbow = (Servo) hardwareMap.get(krElbowName);

        setState(State.DRIVING);

    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }
    public void incrementState(){
        if(state == State.INTAKING)
            state = State.DRIVING;
        else
            state = State.OUTTAKING;
    }
    public void decrementState(){
        if(state == State.OUTTAKING)
            state = State.DRIVING;
        else
            state = State.INTAKING;
    }
    public void setWristPos(double pos){
        wrist.setPosition(pos);
    }
    public void setElbowPos(double pos){
        lElbow.setPosition(pos);
        rElbow.setPosition(1.0 - pos);
    }

    public IntakeSubsystem getIntake(){
        return intake;
    }
    public Servo getWrist(){
        return wrist;
    }
    public void incrementElbowUp(){
        if(lElbow.getPosition() < 1.0) {
            state = State.MANUAL;
            lElbow.setPosition(lElbow.getPosition() + 0.01);
            rElbow.setPosition(rElbow.getPosition() - 0.01);
        }
    }
    public void incrementElbowDown(){
        if(lElbow.getPosition() > 0.0) {
            state = State.MANUAL;
            lElbow.setPosition(lElbow.getPosition() - 0.01);
            rElbow.setPosition(rElbow.getPosition() + 0.01);
        }
    }
    public void incrementWristUp(){
        if(wrist.getPosition() < 1.0) {
            state = State.MANUAL;
            wrist.setPosition(wrist.getPosition() + 0.01);
        }
    }
    public void incrementWristDown(){
        if(wrist.getPosition() > 0.0) {
            state = State.MANUAL;
            wrist.setPosition(wrist.getPosition() - 0.01);
        }
    }

    private void processState(State state) {
        if (this.state == State.DRIVING || this.state == State.INTAKING
                || this.state == State.OUTTAKING) {
            setElbowPos(this.state.elbowPos);
            setWristPos(this.state.wristPos);
            manualElbowPos = state.elbowPos;
            manualWristPos = state.wristPos;
            return;
        }
        else if(this.state == State.MANUAL){
            setElbowPos(manualElbowPos);
            setWristPos(manualWristPos);
        }
    }
    @Override
    public void periodic(Telemetry telemetry) {
        processState(state);

        telemetry.addData("State: ", state);
        telemetry.addData("Left Elbow Position: ", lElbow.getPosition());
        telemetry.addData("Right Elbow Position: ", lElbow.getPosition());
        telemetry.addData("Wrist: ", wrist.getPosition());

//        telemetry.addData("Left Slide Velocity: ", leftSlide.getVelocity());
//        telemetry.addData("Right Slide Velocity: ", rightSlide.getVelocity());
    }


}
