package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FourBarSubsystem {
    private final Servo fourBarLeft, fourBarRight;

    public enum FourBarState {
        DOWN (0.03),
        GLIDE (0.2),
        UP (0.65);

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
        this.fourBarRight = (Servo) hardwareMap.get("fourBarRight");

        this.fourBarLeft.setDirection(Servo.Direction.REVERSE);
        this.fourBarRight.setDirection(Servo.Direction.FORWARD);

        this.state = FourBarState.UP;
    }

    public FourBarSubsystem(Servo fourBarLeft, Servo fourBarRight) {
        this.fourBarLeft = fourBarLeft;
        this.fourBarRight = fourBarRight;

        this.fourBarLeft.setDirection(Servo.Direction.REVERSE);
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
        this.fourBarLeft.setPosition(this.getState().getPosition());
        this.fourBarRight.setPosition(this.getState().getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("FourBarState", this.state);
        telemetry.addData("FourBarPosition", this.state.getPosition());
    }
}
