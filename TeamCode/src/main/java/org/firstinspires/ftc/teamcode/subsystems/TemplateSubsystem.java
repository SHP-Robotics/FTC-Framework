package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class TemplateSubsystem extends Subsystem {
    // Declare devices
    // Example:
    // private final SHPMotor motor;

    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
    }

    private State state;

    public TemplateSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        // motor = new SHPMotor(hardwareMap, "motor");

        // Set initial state
        // Example:
        // setState(State.TOP);
    }

    public void setState(State state) {
        this.state = state;
    }

    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }

    @Override
    public void periodic(Telemetry telemetry) {
        // Add logging if needed
        // Example:
        // telemetry.addData("Motor Encoder: ", motor.getPosition(MotorUnit.TICKS));

        // Handle states
        // Example:
//        switch (state) {
//            case ENABLED:
//                setPower(1.0);
//                break;
//            case DISABLED:
//                setPower(0.0);
//                break;
//        }

        // OR

//        if (state == State.ENABLED) {
//            setPower(1.0);
//        } else if (state == State.DISABLED) {
//            setPower(0.0);
//        }
    }
}
