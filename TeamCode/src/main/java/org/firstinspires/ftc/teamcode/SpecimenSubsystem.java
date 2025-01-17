package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class SpecimenSubsystem {
    public CachingDcMotorEx specimenSlide;
    public static double power = 1.0;

    public enum SpecimenState {
        INTAKE (0),
        HIGH (1700),
        DEPOSIT_HIGH (1200);

        SpecimenState(int position) {
            this.position = position;
        }

        private final int position;

        int getPosition() {
            return this.position;
        }
    }

    private SpecimenState state;

    public SpecimenSubsystem(HardwareMap hardwareMap) {
        this.specimenSlide = new CachingDcMotorEx((DcMotorEx) hardwareMap.get("specimen"));
        this.specimenSlide.setDirection(DcMotor.Direction.REVERSE);

        this.state = SpecimenState.INTAKE;
    }

    public void setMode(DcMotor.RunMode mode) {
        this.specimenSlide.setMode(mode);
    }

    public void setPower(double power) {
        this.specimenSlide.setPowerResult(power);
    }

    public double getPosition() {
        return this.specimenSlide.getCurrentPosition();
    }

    public void setState(SpecimenState state) {
        this.state = state;
    }

    public SpecimenState getState() {
        return this.state;
    }

    public void increment() {
        if (this.getState() == SpecimenSubsystem.SpecimenState.DEPOSIT_HIGH) this.setState(SpecimenSubsystem.SpecimenState.HIGH);
        if (this.getState() == SpecimenSubsystem.SpecimenState.INTAKE) this.setState(SpecimenSubsystem.SpecimenState.HIGH);
    }

    public void decrement() {
        if (this.getState() == SpecimenSubsystem.SpecimenState.DEPOSIT_HIGH) this.setState(SpecimenSubsystem.SpecimenState.INTAKE);
        if (this.getState() == SpecimenSubsystem.SpecimenState.HIGH) this.setState(SpecimenSubsystem.SpecimenState.DEPOSIT_HIGH);
    }

    public boolean isBusy() {
        return this.specimenSlide.isBusy();
    }

    public void init() {
        this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        update();

        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update() {
        this.specimenSlide.setTargetPosition(state.getPosition());
        this.setPower(power);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("SpecimenState", this.state);
        telemetry.addData("SpecimenPosition", this.getPosition());
    }
}
