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



    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
        OPEN,
        CLOSED
    }

    private State leftState;
    private State rightState;


    public ClawSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        // motor = new SHPMotor(hardwareMap, "motor");
        leftClaw = hardwareMap.get(Servo.class, kLeftClawName);
        rightClaw = hardwareMap.get(Servo.class, kRightClawName);

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
    }
    public void openRightClaw() {
        rightClaw.setPosition(kRightClawOpen);
        rightClawPosition = kRightClawOpen;
    }
    public void open(){
        openRightClaw();
        openLeftClaw();
    }

    public void closeLeftClaw() {
        leftClaw.setPosition(kLeftClawClosed);
        leftClawPosition = kLeftClawClosed;
    }
    public void closeRightClaw() {
        rightClaw.setPosition(kRightClawClosed);
        rightClawPosition = kRightClawClosed;
    }
    public void close(){
        closeRightClaw();
        closeLeftClaw();
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
        }

        switch (leftState) {
            case OPEN:
                openRightClaw();
                break;
            case CLOSED:
                closeRightClaw();
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
