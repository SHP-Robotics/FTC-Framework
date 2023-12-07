package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

public class SpeedController {
    SpeedType speedType;

    // If speedType is an override type
    double naturalSpeed;
    double overrideSpeed;

    // If speedType is a gear type
    double gearSpacing;

    // currentSpeed
    double currentSpeed;

    private boolean holdingGearUp = false;
    private boolean holdingGearDown = false;

    private SpeedController(SpeedBuilder speedBuilder) {
        this.speedType = speedBuilder.speedType;

        this.naturalSpeed = clampSpeed(speedBuilder.naturalSpeed);
        this.overrideSpeed = clampSpeed(speedBuilder.overrideSpeed);

        this.gearSpacing = speedBuilder.gearSpacing;

        this.currentSpeed = clampSpeed(speedBuilder.naturalSpeed);
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
                if (DrivingConfiguration.getValue(gamepad, DrivingConfiguration.SPEED_OVERRIDE)) {
                    this.currentSpeed = this.overrideSpeed;
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
            case PID_CONTROLLED_WITH_OVERRIDE:

        }
    }

    public static class SpeedBuilder {
        SpeedType speedType;

        // If speedType is an override type
        double naturalSpeed = 1.0;
        double overrideSpeed = 0.3;

        // If speedType is a gear type
        double gearSpacing = 0.1;

        public SpeedBuilder(SpeedType speedType) {
            this.speedType = speedType;
        }

        public SpeedBuilder setNaturalSpeed(double naturalSpeed) {
            this.naturalSpeed = naturalSpeed;
            return this;
        }

        public SpeedBuilder setOverrideSpeed(double overrideSpeed) {
            this.overrideSpeed = overrideSpeed;
            return this;
        }

        public SpeedBuilder setGearSpacing(double gearSpacing) {
            this.gearSpacing = gearSpacing;
            return this;
        }

        public SpeedController build() {
            return new SpeedController(this);
        }
    }
}