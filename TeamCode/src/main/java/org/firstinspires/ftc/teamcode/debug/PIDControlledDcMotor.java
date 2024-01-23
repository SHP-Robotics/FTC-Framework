package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PIDControlledDcMotor extends DcMotorImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private final double MAX_VELOCITY = 15;

    private ElapsedTime elapsedTime;

    private double lastError = 0;
    private double lastTime = -1;
    private double lastEstimatedVelocity = 0;

    // kP controls topSpeed
    private double kP = 1;
    // kD controls acceleration
    private double kD = -0.005;

    // gamma specifies de-acceleration multiplier vs acceleration (lower is faster de-acceleration)
    // check line 68
    private double gamma = 1;

    public PIDControlledDcMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public PIDControlledDcMotor(DcMotor dcMotor) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        super.setDirection(dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public PIDControlledDcMotor(PIDControlledDcMotorBuilder pidControlledDcMotorBuilder) {
        super(pidControlledDcMotorBuilder.dcMotor.getController(), pidControlledDcMotorBuilder.dcMotor.getPortNumber());
        super.setDirection(pidControlledDcMotorBuilder.dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        this.kP = pidControlledDcMotorBuilder.kP;
        this.kD = pidControlledDcMotorBuilder.kD;
        this.gamma = pidControlledDcMotorBuilder.gamma;
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPower(double power) {
        double targetVelocity = MAX_VELOCITY*power;
        double currentVelocity = this.getVelocity(AngleUnit.RADIANS);
        double error = targetVelocity - currentVelocity;
        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;
        double slope = error - lastError / (deltaTime);

        if (lastEstimatedVelocity != 0 && targetVelocity / lastEstimatedVelocity <= 0) {
            super.setPower(-clamp((error * kP*gamma * deltaTime) + (slope * kD*gamma * deltaTime)));
        } else {
            super.setPower(-clamp((error * kP * deltaTime) + (slope * kD * deltaTime)));
        }

        lastEstimatedVelocity = currentVelocity;

        lastError = error;
        lastTime = time;
    }

    public static class PIDControlledDcMotorBuilder {
        private DcMotor dcMotor;
        private double kP;
        private double kD;
        private double gamma;

        public PIDControlledDcMotorBuilder(DcMotor dcMotor) {
            this.dcMotor = dcMotor;
            this.kP = 1;
            this.kD = -0.005;
            this.gamma = 1;
        }

        public PIDControlledDcMotorBuilder setkP(double kP) {
            this.kP = kP;
            return this;
        }

        public PIDControlledDcMotorBuilder setkD(double kD) {
            this.kD = kD;
            return this;
        }

        public PIDControlledDcMotorBuilder setGamma(double gamma) {
            this.gamma = gamma;
            return this;
        }

        public PIDControlledDcMotor build() {
            return new PIDControlledDcMotor(this);
        }
    }
}