package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class SlideSubsystem {
    public DcMotorEx slideLeft, slideRight;
    public static double power = 0.8;
//    private static final double encoderResolution = 537.689839572; // 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX™ Shaft, 312 RPM, 3.3 - 5V Encoder)
//    private static final double inchesPerRotation = 4.724757455393701; // 2mm Pitch GT2 Hub-Mount Timing Belt Pulley (14mm Bore, 60 Tooth)
    private static final double maxResistance = 1.3; // 12V / 9.2A@12V

    public enum SlideState {
        INTAKE (170),
        LOW (1900),
        HIGH (4200);

        SlideState(int position) {
            this.position = position;
        }

        private final int position;

        int getPosition() {
            return this.position;
        }
    }

    private SlideState state;

    public SlideSubsystem(HardwareMap hardwareMap) {
        this.slideLeft = (DcMotorEx) hardwareMap.get("slideLeft");
        this.slideRight = (DcMotorEx) hardwareMap.get("slideRight");

        this.slideLeft.setDirection(DcMotor.Direction.FORWARD);
        this.slideRight.setDirection(DcMotor.Direction.REVERSE);

        this.state = SlideState.INTAKE;
    }

    public SlideSubsystem(DcMotorEx slideLeft, DcMotorEx slideRight) {
        this.slideLeft = slideLeft;
        this.slideRight = slideRight;

        this.slideLeft.setDirection(DcMotor.Direction.FORWARD);
        this.slideRight.setDirection(DcMotor.Direction.REVERSE);

        this.state = SlideState.INTAKE;
    }

    public void setMode(DcMotor.RunMode mode) {
        this.slideLeft.setMode(mode);
        this.slideRight.setMode(mode);
    }

    public void setPower(double power) {
        this.slideLeft.setPower(power);
        this.slideRight.setPower(power);
    }

    public double getPower() {
        return (Math.abs(this.slideLeft.getPower()) + Math.abs(this.slideRight.getPower())) * 6;
    }

    public double getCurrent() {
        return (Math.abs(this.slideLeft.getCurrent(CurrentUnit.AMPS)) + Math.abs(this.slideRight.getCurrent(CurrentUnit.AMPS))) / 2;
    }

    public double getPosition() {
        return (this.slideLeft.getCurrentPosition() + this.slideRight.getCurrentPosition()) * 1.0 / 2;
    }

    public double getResistance() {
        return this.getPower() * 12 / (this.getCurrent() * maxResistance);
    }

    public void setResistance(double idealResistance) {
        this.slideLeft.setPower((idealResistance * maxResistance * this.getCurrent()) / 12);
        this.slideRight.setPower((idealResistance * maxResistance * this.getCurrent()) / 12);
    }

    public void setState(SlideState state) {
        this.state = state;
    }

    public SlideState getState() {
        return this.state;
    }

    public boolean isBusy() {
        return this.slideLeft.isBusy() || this.slideRight.isBusy();
    }

    public void init() {
        this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        update();

        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update() {
        this.slideLeft.setTargetPosition(state.getPosition());
        this.slideRight.setTargetPosition(state.getPosition());
        this.setPower(power);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("SlideState", this.state);
        telemetry.addData("SlidePosition", this.getPosition());
    }
}
