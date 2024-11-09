package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FourBarSubsystem {
    private Servo fourBarLeft, fourBarRight;

    public enum FourBarState {
        DOWN (0),
        UP (1);

        FourBarState(double position) {
            this.position = position;
        }

        private final double position;

        double getPosition() {
            return this.position;
        }
    }

    private FourBarState state;

    public FourBarSubsystem(HardwareMap hardwareMap) {
        this.fourBarLeft = (Servo) hardwareMap.get("fourBarLeft");
        this.fourBarRight = (Servo) hardwareMap.get("fourBarLeft");

        this.fourBarLeft.setDirection(Servo.Direction.FORWARD);
        this.fourBarRight.setDirection(Servo.Direction.FORWARD);

        this.state = FourBarState.UP;
    }

    public FourBarSubsystem(Servo fourBarLeft, Servo fourBarRight) {
        this.fourBarLeft = fourBarLeft;
        this.fourBarRight = fourBarRight;

        this.fourBarLeft.setDirection(Servo.Direction.FORWARD);
        this.fourBarRight.setDirection(Servo.Direction.FORWARD);

        this.state = FourBarState.UP;
    }

    public void setState(FourBarState state) {
        this.state = state;
    }

    public FourBarState getState() {
        return this.state;
    }

    public void update() {
        this.fourBarLeft.setPosition(state.getPosition());
        this.fourBarRight.setPosition(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("FourBarState", this.state);
        telemetry.addData("FourBarPosition", this.state.getPosition());
    }
}
