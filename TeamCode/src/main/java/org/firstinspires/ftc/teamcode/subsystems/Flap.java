package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.kFlapClose;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kFlapName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kFlapNeutral;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kFlapOpen;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class Flap extends Subsystem {
    // Declare devices
    // Example:
    // private final SHPMotor motor;
    private final Servo flap;

    public enum State {
        // Define states
        // Example:
        OPEN, CLOSE, NEUTRAL
    }

    private State state;

    public Flap(HardwareMap hardwareMap) {
        flap = hardwareMap.get(Servo.class, kFlapName);
        // Set initial state
        // Example:
        setState(State.NEUTRAL);
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
        switch (state) {
            case OPEN:
                flap.setPosition(kFlapOpen);
                break;
            case CLOSE:
                flap.setPosition(kFlapClose);
                break;
            case NEUTRAL:
                flap.setPosition(kFlapNeutral);
                break;
        }
    }
}
