package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

public class AccumulationControlledDcMotor extends DcMotorImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private final double MAX_VELOCITY;
    private boolean usingSimulation = false;
    private double currentVelocity;

    private final ElapsedTime elapsedTime;

    private double lastTime = -1;

    private final double kP;

    // gamma specifies de-acceleration multiplier vs acceleration (lower is faster de-acceleration)
    // check line 68
    private final double gamma;

    public AccumulationControlledDcMotor(AccumulationControlledDcMotorBuilder accumulationControlledDcMotorBuilder) {
        super(accumulationControlledDcMotorBuilder.dcMotor.getController(), accumulationControlledDcMotorBuilder.dcMotor.getPortNumber());
        super.setDirection(accumulationControlledDcMotorBuilder.dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        this.MAX_VELOCITY = accumulationControlledDcMotorBuilder.MAX_VELOCITY;
        this.kP = accumulationControlledDcMotorBuilder.kP;
        this.gamma = accumulationControlledDcMotorBuilder.gamma;
    }

    public void setCurrentVelocity(double currentVelocity) {
        this.usingSimulation = true;
        this.currentVelocity = currentVelocity;
    }

    @Override
    public synchronized double getVelocity() {
        if (!usingSimulation) {
            return super.getVelocity();
        }
        return this.currentVelocity;
    }

    private static double clamp(double x) {
        return Math.max(-1, Math.min(1, x));
    }

    @Override
    public void setPower(double power) {
        if (power == 0) {
            super.setPower(0);
        }

        this.setPower(power, this.getVelocity()!=0 && MAX_VELOCITY*power/this.getVelocity() < 1);
        this.setPower(power, false);
    }

    public void setPower(double power, boolean decelerate) {
        double targetVelocity = MAX_VELOCITY*power;
        if (!usingSimulation) {
            this.currentVelocity = -this.getVelocity(AngleUnit.RADIANS);
        }
        double error = targetVelocity - this.currentVelocity;
        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;

        if (decelerate) {
            super.setPower(clamp(getPower() * gamma + (error * kP * deltaTime)));
//            super.setPower(clamp(getPower() * gamma + (error * kP * gamma * deltaTime)));
        } else {
            super.setPower(clamp(getPower() + (error * kP * deltaTime)));
        }
//
//        if (decelerate) {
//            super.setPower(getPower() * gamma - (error * kP * gamma * deltaTime));
//        } else {
//            super.setPower(getPower() - (error * kP * deltaTime));
//        }

        lastTime = time;
    }

    public static class AccumulationControlledDcMotorBuilder {
        private final DcMotor dcMotor;
        private double MAX_VELOCITY;
        private double kP;
        private double gamma;

        public AccumulationControlledDcMotorBuilder(DcMotor dcMotor) {
            this.dcMotor = dcMotor;
            this.MAX_VELOCITY = Constants.MAX_VELOCITY;
            this.kP = 0.7;
            this.gamma = 1;
        }

        public AccumulationControlledDcMotorBuilder setMaxVelocity(double maxVelocity) {
            this.MAX_VELOCITY = maxVelocity;
            return this;
        }

        public AccumulationControlledDcMotorBuilder setkP(double kP) {
            this.kP = kP;
            return this;
        }

        public AccumulationControlledDcMotorBuilder setGamma(double gamma) {
            this.gamma = gamma;
            return this;
        }

        public AccumulationControlledDcMotor build() {
            return new AccumulationControlledDcMotor(this);
        }
    }
}