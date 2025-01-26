package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class FourBarSubsystem {
    private final CachingServo fourBarLeft, fourBarRight;

    public enum FourBarState {
        DOWN (0.23),
        GLIDE (0.6),
        UP (0.83);

        FourBarState(double position) {
            this.position = position;
        }

        private final double position;

        double getPosition() {
            return this.position;
        }

        public FourBarState increment() {
            if (this == DOWN)
                return GLIDE;
            return UP;
        }

        public FourBarState decrement() {
            if (this == UP)
                return GLIDE;
            return DOWN;
        }
    }

    private FourBarState state;

    public FourBarSubsystem(HardwareMap hardwareMap) {
        this.fourBarLeft = new CachingServo((Servo) hardwareMap.get("left"));
        this.fourBarRight = new CachingServo((Servo) hardwareMap.get("right"));

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
        this.fourBarLeft.setPositionResult(this.getState().getPosition());
        this.fourBarRight.setPositionResult(this.getState().getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("FourBarState", this.state);
        telemetry.addData("FourBarPosition", this.state.getPosition());
    }
}
