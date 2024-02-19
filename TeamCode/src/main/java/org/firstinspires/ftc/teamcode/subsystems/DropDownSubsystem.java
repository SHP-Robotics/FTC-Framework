package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class DropDownSubsystem extends Subsystem {
    Servo dropDown;
    private State state;

    public enum State {
        LOWERED (0),
        SIDE_STACK (0),
        RAISED (0);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    public DropDownSubsystem(HardwareMap hardwareMap) {
        this.dropDown = (Servo) hardwareMap.get(Constants.DropDown.kDropDownName);
        this.state = State.RAISED;
        this.dropDown.setPosition(this.getState().position);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        this.dropDown.setPosition(this.getState().position);

        telemetry.addData("State: ", state);
        telemetry.addData("Drop Down Position: ", this.dropDown.getPosition());
    }
}
