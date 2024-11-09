package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    private CRServo intake;

    public enum IntakeState {
        INTAKE (1),
        OUTAKE (-1),
        NEUTRAL (0);

        IntakeState(double power) {
            this.power = power;
        }

        private final double power;

        double getPower() {
            return this.power;
        }
    }

    private IntakeState state;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.intake = (CRServo) hardwareMap.get("fourBarLeft");
        this.intake.setDirection(CRServo.Direction.FORWARD);
        this.state = IntakeState.NEUTRAL;
    }

    public IntakeSubsystem(CRServo intake) {
        this.intake = intake;
        this.intake.setDirection(CRServo.Direction.FORWARD);
        this.state = IntakeState.NEUTRAL;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return this.state;
    }

    public void update() {
        this.intake.setPower(state.getPower());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("IntakeState", this.state);
        telemetry.addData("IntakePower", this.state.getPower());
    }
}
