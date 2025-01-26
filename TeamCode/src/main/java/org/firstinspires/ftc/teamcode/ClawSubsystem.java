package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class ClawSubsystem {
    private CachingServo claw;
    private ElapsedTime timer;
    private static double CHANGE_SECONDS = 0.5;

    public enum ClawState {
        OPEN (0.2),
        OPENING (0.2),
        CLOSE (0.05),
        CLOSING (0.05);

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
        this.claw = new CachingServo((Servo) hardwareMap.get("claw"));
        this.claw.setDirection(Servo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public void setState(ClawState state) {
        if ((state == ClawState.CLOSING || state == ClawState.OPENING) && state != this.state) {
            timer = new ElapsedTime();
            timer.reset();
        }
        this.state = state;
    }

    public ClawState getState() {
        return this.state;
    }

    public void update() {
        if (this.timer.seconds() > CHANGE_SECONDS) {
            if (this.state == ClawState.CLOSING)
                this.state = ClawState.CLOSE;
            else if (this.state == ClawState.OPENING)
                this.state = ClawState.OPEN;
        }
        this.claw.setPositionResult(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("ClawState", this.state);
        telemetry.addData("ClawPower", this.state.getPosition());
    }
}
