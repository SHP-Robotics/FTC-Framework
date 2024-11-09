package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideSubsystem {
    private DcMotor slideLeft, slideRight;
    private static double power = 0.5;
    private static final double encoderResolution = 537.689839572; // 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX™ Shaft, 312 RPM, 3.3 - 5V Encoder)
    private static final double inchesPerRotation = 4.724757455393701; // 2mm Pitch GT2 Hub-Mount Timing Belt Pulley (14mm Bore, 60 Tooth)

    public enum SlideState {
        INTAKE (0),
        LOW ((int)(25.75 * encoderResolution * inchesPerRotation)),
        HIGH ((int)(43 * encoderResolution * inchesPerRotation));

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
        this.slideLeft = (DcMotor) hardwareMap.get("slideLeft");
        this.slideRight = (DcMotor) hardwareMap.get("slideRight");

        this.slideLeft.setDirection(DcMotor.Direction.FORWARD);
        this.slideRight.setDirection(DcMotor.Direction.FORWARD);

        this.state = SlideState.INTAKE;
    }

    public SlideSubsystem(DcMotor slideLeft, DcMotor slideRight) {
        this.slideLeft = slideLeft;
        this.slideRight = slideRight;

        this.slideLeft.setDirection(DcMotor.Direction.FORWARD);
        this.slideRight.setDirection(DcMotor.Direction.FORWARD);

        this.state = SlideState.INTAKE;
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
        this.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        update();

        this.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.slideLeft.setPower(power);
        this.slideRight.setPower(power);
    }

    public void update() {
        this.slideLeft.setTargetPosition(state.getPosition());
        this.slideRight.setTargetPosition(state.getPosition());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("SlideState", this.state);
        telemetry.addData("SlidePosition", this.state.getPosition());
    }
}
