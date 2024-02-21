package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class DropDownSubsystem extends Subsystem {
    Servo dropdown;
    private State state;

    public enum State {
        RAISED (0.67),
        FIVE_HEIGHT (0.73),
        FOUR_HEIGHT (0.7575),
        THREE_HEIGHT (0.785),
        TWO_HEIGHT (0.8125),
        GROUND_HEIGHT (0.84);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    public DropDownSubsystem(HardwareMap hardwareMap) {
        this.dropdown = (Servo) hardwareMap.get(Constants.DropDown.kDropdownName);
        this.state = State.RAISED;
        this.dropdown.setPosition(this.getState().position);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        this.dropdown.setPosition(this.getState().position);

        telemetry.addData("State: ", state);
        telemetry.addData("Drop Down Position: ", this.dropdown.getPosition());
    }
}
