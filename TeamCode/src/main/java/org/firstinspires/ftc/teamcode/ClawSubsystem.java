package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem {
    private Servo intake;

    public enum ClawState {
        OPEN (0.25),
        CLOSE (0.06);

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
        this.intake = (Servo) hardwareMap.get("intake");
        this.intake.setDirection(Servo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public ClawSubsystem(Servo intake) {
        this.intake = intake;
        this.intake.setDirection(Servo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public void setState(ClawState state) {
        this.state = state;
    }

    public ClawState getState() {
        return this.state;
    }

    public void update() {
        this.intake.setPosition(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("ClawState", this.state);
        telemetry.addData("ClawPower", this.state.getPosition());
    }
}
