package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.DrivingConfiguration;

public class SpeedController {
    public enum SpeedType {
        NO_CHANGE,
        SINGLE_OVERRIDE,
        DOUBLE_OVERRIDE,
        GEAR_SHIFT
    }

    SpeedType speedType;

    // If speedType is an override type
    double naturalSpeed;
    double overrideSpeedOne;
    double overrideSpeedTwo;

    // If speedType is a gear type
    double gearSpacing;

    // currentSpeed
    double currentSpeed;

    // Whether or not to coast
    boolean applyMinimumVoltage;

    private boolean holdingGearUp = false;
    private boolean holdingGearDown = false;

    private ElapsedTime timer;
    private double dt;
    private double lastTime = -1;
    private PIDController speedPIDController = new PIDController(0.02, 0, 0);

    private SpeedController(SpeedBuilder speedBuilder) {
        this.speedType = speedBuilder.speedType;

        this.naturalSpeed = clampSpeed(speedBuilder.naturalSpeed);
        this.overrideSpeedOne = clampSpeed(speedBuilder.overrideSpeedOne);
        this.overrideSpeedTwo = clampSpeed(speedBuilder.overrideSpeedTwo);

        this.gearSpacing = speedBuilder.gearSpacing;

        this.currentSpeed = clampSpeed(speedBuilder.naturalSpeed);

        this.applyMinimumVoltage = speedBuilder.applyMinimumVoltage;

        timer = new ElapsedTime();
        timer.reset();
    }

    public double clampSpeed(double speed) {
        return Math.max(-1, Math.min(1, speed));
    }

    public double getSpeed() {
        return this.currentSpeed;
    }

    public void setSpeed(double speed) {
        this.currentSpeed = clampSpeed(speed);
    }

    public void updateSpeed(Gamepad gamepad) {
        switch (this.speedType) {
            case SINGLE_OVERRIDE:
                if (DrivingConfiguration.getValue(gamepad, DrivingConfiguration.SPEED_OVERRIDE_ONE) > 0) {
                    this.currentSpeed = this.overrideSpeedOne;
                } else {
                    this.currentSpeed = this.naturalSpeed;
                }

                break;

            case DOUBLE_OVERRIDE:
                if (DrivingConfiguration.getValue(gamepad, DrivingConfiguration.SPEED_OVERRIDE_ONE) > 0) {
                    this.currentSpeed = this.overrideSpeedOne;
                } else if (DrivingConfiguration.getValue(gamepad, DrivingConfiguration.SPEED_OVERRIDE_TWO) > 0) {
                    this.currentSpeed = this.overrideSpeedTwo;
                } else {
                    this.currentSpeed = this.naturalSpeed;
                }

                break;
            case GEAR_SHIFT:
                if (DrivingConfiguration.getValue(gamepad, DrivingConfiguration.GEAR_UP)) {
                    if (!holdingGearUp) {
                        this.currentSpeed = clampSpeed(this.currentSpeed + this.gearSpacing);
                    }
                    holdingGearUp = true;
                } else {
                    holdingGearUp = false;
                }

                if (DrivingConfiguration.getValue(gamepad, DrivingConfiguration.GEAR_DOWN)) {
                    if (!holdingGearDown) {
                        this.currentSpeed = clampSpeed(this.currentSpeed - this.gearSpacing);
                    }
                    holdingGearDown = true;
                } else {
                    holdingGearDown = false;
                }

                break;
        }
    }

    public static class SpeedBuilder {
        SpeedType speedType;

        // If speedType is an override type
        double naturalSpeed = 1.0;
        double overrideSpeedOne = 0.6;
        double overrideSpeedTwo = 0.3;

        // If speedType is a gear type
        double gearSpacing = 0.1;

        boolean applyMinimumVoltage = false;

        public SpeedBuilder(SpeedType speedType) {
            this.speedType = speedType;
        }

        public SpeedBuilder setNaturalSpeed(double naturalSpeed) {
            this.naturalSpeed = naturalSpeed;
            return this;
        }

        public SpeedBuilder setOverrideOneSpeed(double overrideSpeedOne) {
            this.overrideSpeedOne = overrideSpeedOne;
            return this;
        }

        public SpeedBuilder setOverrideTwoSpeed(double overrideSpeedTwo) {
            this.overrideSpeedOne = overrideSpeedTwo;
            return this;
        }

        public SpeedBuilder setGearSpacing(double gearSpacing) {
            this.gearSpacing = gearSpacing;
            return this;
        }

        public SpeedBuilder setApplyMinimumVoltage(boolean applyMinimumVoltage) {
            this.applyMinimumVoltage = applyMinimumVoltage;
            return this;
        }

        public SpeedController build() {
            return new SpeedController(this);
        }
    }
}