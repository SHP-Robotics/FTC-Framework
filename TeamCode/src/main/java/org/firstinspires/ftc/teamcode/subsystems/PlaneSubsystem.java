package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Plane.kPlaneLaunch;
import static org.firstinspires.ftc.teamcode.Constants.Plane.kPlaneLoad;
import static org.firstinspires.ftc.teamcode.Constants.Plane.kPlaneName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class PlaneSubsystem extends Subsystem {
    // Declare devices
    // Example:
    private final Servo planeServo;
    public enum State {
        // Define states
        // Example:
        LAUNCH,
        LOAD
    }

    private State state;

    public PlaneSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:

        planeServo = hardwareMap.get(Servo.class, kPlaneName);
        state = State.LOAD;
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
            case LOAD:
                planeServo.setPosition(kPlaneLoad);
                break;
            case LAUNCH:
                planeServo.setPosition(kPlaneLaunch);
                break;
        }
        telemetry.addData("Plane Position: ", planeServo.getPosition());
    }
}