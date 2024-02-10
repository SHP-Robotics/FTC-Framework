package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AccumulationControlledDcMotor extends DcMotorImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private final double MAX_VELOCITY = 12;

    private ElapsedTime elapsedTime;

    private double lastTime = -1;

    private double kP = 0.7;

    // gamma specifies de-acceleration multiplier vs acceleration (lower is faster de-acceleration)
    // check line 68
    private double gamma = 1;

    public AccumulationControlledDcMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public AccumulationControlledDcMotor(DcMotor dcMotor) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        super.setDirection(dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public AccumulationControlledDcMotor(AccumulationControlledDcMotorBuilder accumulationControlledDcMotorBuilder) {
        super(accumulationControlledDcMotorBuilder.dcMotor.getController(), accumulationControlledDcMotorBuilder.dcMotor.getPortNumber());
        super.setDirection(accumulationControlledDcMotorBuilder.dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        this.kP = accumulationControlledDcMotorBuilder.kP;
        this.gamma = accumulationControlledDcMotorBuilder.gamma;
    }

    public void resetEncoders() {
        this.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPower(double power, boolean decelerate) {
        double targetVelocity = MAX_VELOCITY*power;
        double currentVelocity = -this.getVelocity(AngleUnit.RADIANS);
        double error = targetVelocity - currentVelocity;
        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;

        if (decelerate) {
            super.setPower(clamp(getPower() * gamma - (error * kP * gamma * deltaTime)));
        } else {
            super.setPower(clamp(getPower() - (error * kP * deltaTime)));
        }

        lastTime = time;
    }

    public static class AccumulationControlledDcMotorBuilder {
        private DcMotor dcMotor;
        private double kP;
        private double gamma;

        public AccumulationControlledDcMotorBuilder(DcMotor dcMotor) {
            this.dcMotor = dcMotor;
            this.kP = 0.7;
            this.gamma = 1;
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