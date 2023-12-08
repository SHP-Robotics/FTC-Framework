package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kClawName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kIntakeName;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class ClawSubsystem extends Subsystem {
    // Declare devices
    // Example:
    private final Servo claw;
    //private final Servo pitch;

    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
        PICKUP,
        DROP,
        CARRY
    }

    private State state;

    public ClawSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        claw = hardwareMap.get(Servo.class, kClawName);
        //pitch = hardwareMap.get(Servo.class,"pitch");

        // Set initial state
        // Example:
        setState(State.PICKUP);
    }

    public void setState(State state) {
        this.state = state;
    }

    public String getState(){ return state.toString(); }
    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }

    @Override
    public void periodic(Telemetry telemetry) {
        // Add logging if needed
        // Example:
        telemetry.addData("Intake State: ", getState());
        telemetry.update();

        // Handle states
        // Example:
        switch (state) {
            case PICKUP:
                claw.setPosition(0.6);
                //pitch.setPosition(0);
                break;
            case DROP:
                claw.setPosition(0.25);
                //pitch.setPosition(1);
                break;
            case CARRY:
                //claw.setPosition(0.5);
                //pitch.setPosition(0.5);
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