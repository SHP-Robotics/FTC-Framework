package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.kPixelServo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class PixelServo extends Subsystem {
    // Declare devices
    // Example:
    private final Servo pixelServo;
    public enum State {
        // Define states
        // Example:
        IN, OUT
        // In .setPower(1)
        // Out .setPower(-1)
        // Disable .setPower(0)
    }

    private State state;

    public PixelServo(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:

        pixelServo = hardwareMap.get(Servo.class, kPixelServo);

        state = State.IN;
    }


    public void setState(State state) {
        this.state = state;
    }
    public State getState() {
        return state;
    }

    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }

    @Override
    public void periodic(Telemetry telemetry) {
        switch (state) {
            case IN:
                pixelServo.setPosition(1.0);
                break;
            case OUT:
                pixelServo.setPosition(0.5);
                break;
        }
    }
}
