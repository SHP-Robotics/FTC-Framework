package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.algorithms.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class SpecimenSlideSubsystem {
    public CachingDcMotorEx specimenSlide;
//    public static double power = 1.0;
    public static PID powerPID = new PID(0.003, 0, 0);

    public enum SpecimenState {
        INTAKE (10),
        BELOW_HIGH (1250),
        HIGH (1833);

        SpecimenState(int position) {
            this.position = position;
        }

        private final int position;

        int getPosition() {
            return this.position;
        }
    }

    private SpecimenState state;

    public SpecimenSlideSubsystem(HardwareMap hardwareMap) {
        this.specimenSlide = new CachingDcMotorEx((DcMotorEx) hardwareMap.get("specimen"));
        this.specimenSlide.setDirection(DcMotor.Direction.REVERSE);
        this.specimenSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.specimenSlide.setCachingTolerance(0.05);

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
        if (this.getState() == SpecimenState.BELOW_HIGH) this.setState(SpecimenSlideSubsystem.SpecimenState.HIGH);
        if (this.getState() == SpecimenSlideSubsystem.SpecimenState.INTAKE) this.setState(SpecimenSlideSubsystem.SpecimenState.HIGH);
    }

    public void decrement() {
        if (this.getState() == SpecimenSlideSubsystem.SpecimenState.BELOW_HIGH) this.setState(SpecimenSlideSubsystem.SpecimenState.INTAKE);
        if (this.getState() == SpecimenSlideSubsystem.SpecimenState.HIGH) this.setState(SpecimenState.BELOW_HIGH);
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
        this.setPower(powerPID.getOutput(this.specimenSlide.getCurrentPosition(), state.getPosition()));
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("SpecimenState", this.state);
        telemetry.addData("SpecimenPower", this.specimenSlide.getPower());
        telemetry.addData("SpecimenPosition", this.getPosition());
    }
}
