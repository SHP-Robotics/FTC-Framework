package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class SpecimenClawSubsystem {
    private final CachingServo claw;

    public enum ClawState {
        OPEN (0.62),
        CLOSE (0.958);

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
        this.claw = new CachingServo((Servo) hardwareMap.get("claw"));
        this.claw.setDirection(Servo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public void setState(ClawState state) {
        this.state = state;
    }

    public ClawState getState() {
        return this.state;
    }

    public void update() {
        this.claw.setPositionResult(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("ClawState", this.state);
        telemetry.addData("ClawPower", this.state.getPosition());
    }
}
