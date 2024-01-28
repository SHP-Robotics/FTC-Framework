package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Claw.kLeftClawClosed;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kLeftClawName;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kLeftClawOpen;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kRightClawClosed;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kRightClawName;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kRightClawOpen;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ClawSubsystem extends Subsystem {
    // Declare devices
    // Example:
     private final Servo leftClaw;
     private final Servo rightClaw;
     private double leftClawPosition;
     private double rightClawPosition;
     private double leftManual =0;
    private double rightManual =0;


    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
        OPEN,
        CLOSED,
        MANUAL
    }

    private State leftState;
    private State rightState;


    public ClawSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        // motor = new SHPMotor(hardwareMap, "motor");
        leftClaw = hardwareMap.get(Servo.class, kLeftClawName);
        rightClaw = hardwareMap.get(Servo.class, kRightClawName);

        setRightState(State.CLOSED);
        setLeftState(State.CLOSED);
        // Set initial state
        // Example:
        // setState(State.TOP);
    }

    public void setRightState(State state) {
        this.rightState = state;
    }
    public void setLeftState(State state) {
        this.leftState = state;
    }


    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }
    public void openLeftClaw() {
        leftClaw.setPosition(kLeftClawOpen);
        leftClawPosition = kLeftClawOpen;
//        setLeftState(State.OPEN);
    }
    public void openRightClaw() {
        rightClaw.setPosition(kRightClawOpen);
        rightClawPosition = kRightClawOpen;
//        setRightState(State.OPEN);
    }
    public void open(){
        openRightClaw();
        setRightState(State.OPEN);
        openLeftClaw();
        setLeftState(State.OPEN);

    }

    public void closeLeftClaw() {
        leftClaw.setPosition(kLeftClawClosed);
        leftClawPosition = kLeftClawClosed;
//        setLeftState(State.CLOSED);
    }
    public void closeRightClaw() {
        rightClaw.setPosition(kRightClawClosed);
        rightClawPosition = kRightClawClosed;
//        setRightState(State.CLOSED);
    }
    public void close(){
        closeRightClaw();
        setLeftState(State.CLOSED);
        closeLeftClaw();
        setRightState(State.CLOSED);
    }
    public double upLeft(){
        setLeftState(State.MANUAL);
        leftManual += 0.01;
        return leftManual;
    }
    public double downLeft(){
        setLeftState(State.MANUAL);
        leftManual -= 0.01;
        return leftManual;
    }
    public double upRight(){
        setRightState(State.MANUAL);
        rightManual += 0.01;
        return rightManual;
    }
    public double downRight(){
        setRightState(State.MANUAL);
        rightManual -= 0.01;
        return rightManual;
    }
    public void nextLeft(){
        if(getLeftState() == State.CLOSED)
            setLeftState(State.OPEN);
        else if(getLeftState() == State.OPEN)
            setLeftState(State.CLOSED);
    }
    public void nextRight(){
        if(getRightState() == State.CLOSED)
            setRightState(State.OPEN);
        else if(getRightState() == State.OPEN)
            setRightState(State.CLOSED);
    }

    public State getLeftState() {
        return leftState;
    }
    public State getRightState() {
        return rightState;
    }

    public void nextState(){
        if(getRightState() == State.OPEN
            && getRightState() == State.OPEN){
            leftState = State.CLOSED;
            rightState = State.CLOSED;
        }
        else {
            leftState = State.OPEN;
            rightState = State.OPEN;
        }
    }

    public boolean closed(){
        return getRightState() == State.CLOSED && getLeftState() == State.CLOSED;
    }


    @Override
    public void periodic(Telemetry telemetry) {
        // Add logging if needed
        // Example:
        telemetry.addData("Left Claw: ", getLeftState());
        telemetry.addData("Right Claw: ", getRightState());

        switch (leftState) {
            case OPEN:
                openLeftClaw();
                break;
            case CLOSED:
                closeLeftClaw();
                break;
            case MANUAL:
                rightClaw.setPosition(rightManual);
                break;
        }

        switch (leftState) {
            case OPEN:
                openRightClaw();
                break;
            case CLOSED:
                closeRightClaw();
                break;
            case MANUAL:
                leftClaw.setPosition(leftManual);
                break;
        }

        // OR

//        if (state == State.ENABLED) {
//            setPower(1.0);
//        } else if (state == State.DISABLED) {
//            setPower(0.0);
//        }
    }
}
