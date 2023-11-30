package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.kIntakeName;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class IntakeSubsystem extends Subsystem {
    // Declare devices
    // Example:
     private final SHPMotor intake;

    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
        INTAKING,
        OUTTAKING,
        PAUSED
    }

    private State state;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
         intake = new SHPMotor(hardwareMap, kIntakeName);

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
            case INTAKING:
                intake.setPower(0.6);
                break;
            case OUTTAKING:
                intake.setPower(-0.6);
                break;
            case PAUSED:
                intake.setPower(0.0);
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
