package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.WristSubsystem.WristState.DOWN;
import static org.firstinspires.ftc.teamcode.WristSubsystem.WristState.UP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystem {
    public Servo wrist;

    public enum WristState {
        DOWN (0.62),
        UP (0.0);

        WristState(double position) {
            this.position = position;
        }

        private final double position;

        double getPosition() {
            return this.position;
        }
    }

    private WristState state;

    public WristSubsystem(HardwareMap hardwareMap) {
        this.wrist = (Servo) hardwareMap.get("wrist");
        this.wrist.setDirection(Servo.Direction.FORWARD);
        this.state = UP;
    }

    public void setState(WristState state) {
        this.state = state;
    }

    public WristState getState() {
        return this.state;
    }

    public void toggle() {
        if (this.state == DOWN)
            this.state = UP;
        else
            this.state = DOWN;
    }

    public void update() {
        this.wrist.setPosition(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("WristState", this.state);
        telemetry.addData("WristPosition", this.state.getPosition());
    }
}
