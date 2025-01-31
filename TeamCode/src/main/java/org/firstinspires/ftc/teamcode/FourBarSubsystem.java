package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.DOWN;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.GLIDE;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.UP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class FourBarSubsystem {
    private final CachingServo fourBarLeft, fourBarRight;

    public enum FourBarState {
        UP (0.33),
        GLIDE (0.70),
        DOWN (0.86);

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
        this.fourBarLeft = new CachingServo((Servo) hardwareMap.get("left"));
        this.fourBarRight = new CachingServo((Servo) hardwareMap.get("right"));

        this.fourBarLeft.setDirection(Servo.Direction.REVERSE);
        this.fourBarRight.setDirection(Servo.Direction.FORWARD);

        this.state = UP;
    }

    public void setState(FourBarState state) {
        this.state = state;
    }

    public FourBarState getState() {
        return this.state;
    }

    public void increment() {
        if (state == DOWN)
            state = GLIDE;
        else
            state = UP;
    }

    public void decrement() {
        if (state == UP)
            state = GLIDE;
        else
            state = DOWN;
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
