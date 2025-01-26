package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class WristSubsystem {
    private CachingServo wrist;

    public enum WristState {
        DEPOSIT(0.5),
        UP(1);

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
        this.wrist = new CachingServo((Servo) hardwareMap.get("wrist"));
        this.wrist.setDirection(Servo.Direction.FORWARD);
        this.state = WristState.UP;
    }

    public void setState(WristState state) {
        this.state = state;
    }

    public WristState getState() {
        return this.state;
    }

    public void update() {
        this.wrist.setPositionResult(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("WristState", this.state);
        telemetry.addData("WristPosition", this.state.getPosition());
    }
}
