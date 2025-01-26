package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpecimenClawSubsystem {
    public Servo specimenClaw;

    public enum ClawState {
        OPEN (0.5),
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

    public SpecimenClawSubsystem(HardwareMap hardwareMap) {
        this.specimenClaw = (Servo) hardwareMap.get("specimenClaw");
        this.specimenClaw.setDirection(Servo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public SpecimenClawSubsystem(Servo sampleClaw) {
        this.specimenClaw = sampleClaw;
        this.specimenClaw.setDirection(Servo.Direction.FORWARD);
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
        this.specimenClaw.setPosition(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("ClawState", this.state);
        telemetry.addData("ClawPower", this.state.getPosition());
    }
}
