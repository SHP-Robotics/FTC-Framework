package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem {
    public Servo claw;

    public enum ClawState {
        OPEN (0.68),
        CLOSE (0.32);

        ClawState(double position) {
            this.position = position;
        }

        private final double position;

        double getPosition() {
            return this.position;
        }
    }

    private ClawState state;

    public ClawSubsystem(HardwareMap hardwareMap) {
        this.claw = (Servo) hardwareMap.get("claw");
        this.claw.setDirection(Servo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public ClawSubsystem(Servo claw) {
        this.claw = claw;
        this.claw.setDirection(Servo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public void setState(ClawState state) {
        this.state = state;
    }

    public ClawState getState() {
        return this.state;
    }

    public void toggle() {
        if (this.state == ClawState.OPEN)
            this.state = ClawState.CLOSE;
        else
            this.state = ClawState.OPEN;
    }

    public void update() {
        this.claw.setPosition(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("ClawState", this.state);
        telemetry.addData("ClawPower", this.state.getPosition());
    }
}
