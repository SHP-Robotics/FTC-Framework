package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.kIntakeName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kLiftName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class LiftSubsystem extends Subsystem {
    // Declare devices
    // Example:
    private final SHPMotor lift;

    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
        RELEASING,
        LIFTING,
        PAUSED,
        CARRY
    }

    private State state;

    public LiftSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        lift = new SHPMotor(hardwareMap, kLiftName);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.resetEncoder();
        lift.reverseDirection();
        // Set initial state
        // Example:
        setState(State.PAUSED);
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

        // Handle states
        // Example:
        switch (state) {
            case LIFTING:
                lift.setPower(0.5);
                break;
            case PAUSED:
                lift.setPower(0);
                break;
            case RELEASING:
                lift.setPower(-1);
                break;
            case CARRY:
                lift.setPower(-0.5);
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