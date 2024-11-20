package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem {
    private CRServo intake;

    public enum ClawState {
        OPEN (1),
        CLOSE (0);

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
        this.intake = (CRServo) hardwareMap.get("intake");
        this.intake.setDirection(CRServo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public ClawSubsystem(CRServo intake) {
        this.intake = intake;
        this.intake.setDirection(CRServo.Direction.FORWARD);
        this.state = ClawState.CLOSE;
    }

    public void setState(ClawState state) {
        this.state = state;
    }

    public ClawState getState() {
        return this.state;
    }

    public void update() {
        this.intake.setPower(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("ClawState", this.state);
        telemetry.addData("ClawPower", this.state.getPosition());
    }
}
