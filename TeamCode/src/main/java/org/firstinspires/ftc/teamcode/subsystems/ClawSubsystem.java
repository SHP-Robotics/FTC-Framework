package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ClawSubsystem extends Subsystem {
//    // Declare devices
//    // Example:
     private final Servo leftWrist;
     private final Servo rightWrist;

     private final Servo squeeze;
     private double leftClawPosition;
     private double rightClawPosition;
//     private double leftManual =0;
//    private double rightManual =0;
//
//
    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
        DOWN,
        DRIVE,

        MID,
        DEPOSIT,

        INIT,

    }
//
    private State state;
    private boolean clawOpen;
//
//
    public ClawSubsystem(HardwareMap hardwareMap) {
        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");

        squeeze = hardwareMap.get(Servo.class, "claw");

        clawOpen = false;



        state = State.DOWN;
    }

    public void closeClaw() {
        clawOpen = false;
    }

    public void openClaw() {
        clawOpen = true;
    }

    public boolean isOpen() {
        return clawOpen;
    }

    public String getSqueezeState() {
        return clawOpen ? "OPEN" : "CLOSE";
    }

    public void toggleClaw() { clawOpen = !clawOpen; }
    public void setState(State state) {
        this.state = state;
    }

    public String getState() {
        return state.toString();
    }

    private void setDualPos(double position) {
        leftWrist.setPosition(position);
        rightWrist.setPosition(1-position);
    }

    public void incrementState() {
        if (state == State.DOWN) { setState(State.DRIVE); return; }
        if (state == State.DRIVE) { setState(State.DEPOSIT); return; }
    }

    public void deincrementState() {
        if (state == State.DEPOSIT) { setState(State.DRIVE); return; }
        if (state == State.DRIVE) { setState(State.DOWN); return; }
    }


    private void processState() {
        switch (state) {
            case INIT:
                setDualPos(Constants.Claw.kWristInit);
                break;
            case DOWN:
                setDualPos(Constants.Claw.kWristDown);
                break;
            case DRIVE:
                setDualPos(Constants.Claw.kWristDrive);
                break;
            case DEPOSIT:
                setDualPos(Constants.Claw.kWristDeposit);
            case MID:
                setDualPos(Constants.Claw.kWristDeposit);

        }

    }

    private void processClaw() {
        if (clawOpen) {
            squeeze.setPosition(Constants.Claw.kClawOpen);
        } else {
            squeeze.setPosition(Constants.Claw.kClawClosed);
        }
    }
    @Override
    public void periodic(Telemetry telemetry) {

        processState();
        processClaw();

    }
}
