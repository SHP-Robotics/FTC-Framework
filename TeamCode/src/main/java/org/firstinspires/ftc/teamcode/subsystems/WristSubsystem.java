package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.kAdjustHolder;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class WristSubsystem extends Subsystem {
    // Declare devices
    // Example:
    private final Servo adjustHolder;
    public enum State {
        UP, DOWN,HALFWAY

    }

    private State state;

    public WristSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:

        adjustHolder = hardwareMap.get(Servo.class, kAdjustHolder);

        state = State.DOWN;
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

            case DOWN:
                adjustHolder.setPosition(0.625);
                break;
            case UP:
                adjustHolder.setPosition(0.3);
                break;
            case HALFWAY:
                adjustHolder.setPosition(1);
                break;


        }
        telemetry.addData("Position; ", adjustHolder.getPosition());
    }
}
