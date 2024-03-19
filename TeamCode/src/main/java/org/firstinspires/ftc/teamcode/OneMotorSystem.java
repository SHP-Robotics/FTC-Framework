package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OneMotorSystem {
    private DcMotor motor;

    private double power = 0.7;
    private double staticPower;

    private boolean shouldUseEncoders;
    private boolean shouldUseBrakes;
    private boolean isStatic = false;
    private double staticPosition = 0;

    public OneMotorSystem(OneMotorSystemBuilder oneMotorSystemBuilder) {
        this.motor = oneMotorSystemBuilder.motor;
        this.staticPower = oneMotorSystemBuilder.staticPower;

        this.shouldUseBrakes = oneMotorSystemBuilder.shouldUseBrakes;
        this.shouldUseEncoders = oneMotorSystemBuilder.shouldUseEncoders;

        if (this.shouldUseEncoders) {
            this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.motor.setDirection(oneMotorSystemBuilder.direction);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setLiftPower(double power) {
        this.power = power;
    }

    public void applyBrakes() {
        staticPosition = motor.getCurrentPosition();
        motor.setTargetPosition((int) staticPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(staticPower);
        isStatic = true;
    }

    public void drive(double power) {
        if (power == 0) {
            if (!isStatic) {
                if (shouldUseBrakes) {
                    applyBrakes();
                } else {
                    motor.setPower(0);
                }
            }
        } else {
            if (isStatic) {
                if (this.shouldUseEncoders) {
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                isStatic = false;
            }
            motor.setPower(power * this.power);
        }
    }

    public void deactivate() {
        motor.setPower(0);
    }

    public void setPosition(int position, boolean wait) {
        motor.setTargetPosition((int) (position));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        if (wait) {
            while (motor.isBusy()) {}
        }
    }

    public static class OneMotorSystemBuilder {
        DcMotor motor;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        boolean shouldUseEncoders;
        boolean shouldUseBrakes = false;
        double staticPower = 0;

        public OneMotorSystemBuilder(HardwareMap hardwareMap, String motorName) {
            this.motor = hardwareMap.get(DcMotor.class, motorName);
        }

        public OneMotorSystemBuilder setDirection(DcMotorSimple.Direction direction) {
            this.direction = direction;
            return this;
        }

        public OneMotorSystemBuilder setUseBrakes(boolean shouldUseBrakes) {
            this.shouldUseBrakes = shouldUseBrakes;
            return this;
        }

        public OneMotorSystemBuilder setUseEncoders(boolean shouldUseEncoders) {
            this.shouldUseEncoders = shouldUseEncoders;
            return this;
        }

        public OneMotorSystemBuilder setStaticPower(double staticPower) {
            this.staticPower = staticPower;
            return this;
        }

        public OneMotorSystem build() {
            return new OneMotorSystem(this);
        }
    }
}
