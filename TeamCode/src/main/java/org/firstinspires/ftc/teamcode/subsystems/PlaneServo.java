package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Plane.kPlaneServo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class PlaneServo extends Subsystem {
    // Declare devices
    // Example:
    private final Servo planeServo;
    public enum State {
        // Define states
        // Example:
        IN, OUT
        // In .setPower(1)
        // Out .setPower(-1)
        // Disable .setPower(0)
    }

    private State state;

    public PlaneServo(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:

        planeServo = hardwareMap.get(Servo.class, kPlaneServo);

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
                planeServo.setPosition(0); //0
                break;
            case OUT:
                planeServo.setPosition(0.5);
                break;
        }
        telemetry.addData("State: ", state);
        telemetry.addData("Position: ", planeServo.getPosition());
    }
}
