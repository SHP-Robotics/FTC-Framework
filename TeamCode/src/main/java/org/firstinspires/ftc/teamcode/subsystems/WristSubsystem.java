package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kAdjustHolder;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWristClimb;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWristDown;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWristHalfway;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWristMosaic;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWristStageDoor;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWristUp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class WristSubsystem extends Subsystem {
    // Declare devices
    // Example:
    private final Servo adjustHolder;
    public enum State {
        UP, DOWN,HALFWAY, CLIMB, STAGE_DOOR, MOSAIC

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
                adjustHolder.setPosition(kWristDown);
                break;
            case UP:
                adjustHolder.setPosition(kWristUp);
                break;
            case HALFWAY:
                adjustHolder.setPosition(kWristHalfway);
                break;
            case CLIMB:
                adjustHolder.setPosition(kWristClimb);
                break;
            case STAGE_DOOR:
                adjustHolder.setPosition(kWristStageDoor);
                break;
            case MOSAIC:
                adjustHolder.setPosition(kWristMosaic);
        }
        telemetry.addData("Position; ", adjustHolder.getPosition());
    }
}
