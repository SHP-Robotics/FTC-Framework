package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SampleSlideSubsystem.SlideState.HIGH;
import static org.firstinspires.ftc.teamcode.SampleSlideSubsystem.SlideState.INTAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class SampleSlideSubsystem {
    public CachingDcMotorEx slideLeft, slideRight;
    public static double power = 1.0;

    public enum SlideState {
        INTAKE (170),
        HIGH (4050);

        SlideState(int position) {
            this.position = position;
        }

        private final int position;

        int getPosition() {
            return this.position;
        }
    }

    private SlideState state;

    public SampleSlideSubsystem(HardwareMap hardwareMap) {
        this.slideLeft = new CachingDcMotorEx((DcMotorEx) hardwareMap.get("slideLeft"));
        this.slideRight = new CachingDcMotorEx((DcMotorEx) hardwareMap.get("slideRight"));

        this.slideLeft.setDirection(DcMotor.Direction.FORWARD);
        this.slideRight.setDirection(DcMotor.Direction.REVERSE);

        this.state = INTAKE;
    }

    public void setMode(DcMotor.RunMode mode) {
        this.slideLeft.setMode(mode);
        this.slideRight.setMode(mode);
    }

    public void setPower(double power) {
        this.slideLeft.setPowerResult(power);
        this.slideRight.setPowerResult(power);
    }

    public double getPosition() {
        return (this.slideLeft.getCurrentPosition() + this.slideRight.getCurrentPosition()) * 1.0 / 2;
    }

    public void setState(SlideState state) {
        this.state = state;
    }

    public SlideState getState() {
        return this.state;
    }

    public void increment() {
        state = HIGH;
    }

    public void decrement() {
        state = INTAKE;
    }

    public boolean isBusy() {
        return this.slideLeft.isBusy() || this.slideRight.isBusy();
    }

    public double getPower() {
        return (this.slideLeft.getPower() + this.slideRight.getPower()) / 2;
    }

    public void init() {
        this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        update();

        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setPower(power);
    }

    public void update() {
        this.slideLeft.setTargetPosition(state.getPosition());
        this.slideRight.setTargetPosition(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("SlideState", this.state);
        telemetry.addData("SlidePosition", this.getPower());
        telemetry.addData("SlidePosition", this.getPosition());
        telemetry.addData("SlidePositionL", this.slideLeft.getCurrentPosition());
        telemetry.addData("SlidePositionR", this.slideRight.getCurrentPosition());
    }
}
