package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PIDControlledDcMotor extends DcMotorImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private final double MAX_VELOCITY = 12;

    private ElapsedTime elapsedTime;

    private double lastError = 0;
    private double lastTime = -1;

    // the cubic multiplier for the kP function
    private double kC = 0;
    // the linear multiplier for the kP function
    private double kB = 0.7;

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

        this.kC = pidControlledDcMotorBuilder.kC;
        this.kB = pidControlledDcMotorBuilder.kB;
        this.gamma = pidControlledDcMotorBuilder.gamma;
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

        double kCT = (error * error * kC) + kB;

        if (decelerate) {
            super.setPower(clamp(getPower() * gamma - (error * kCT * gamma * deltaTime)));
        } else {
            super.setPower(clamp(getPower() - (error * kCT * deltaTime)));
        }

        lastError = error;
        lastTime = time;
    }

    public static class PIDControlledDcMotorBuilder {
        private DcMotor dcMotor;
        private double kC;
        private double kB;
        private double gamma;

        public PIDControlledDcMotorBuilder(DcMotor dcMotor) {
            this.dcMotor = dcMotor;
            this.kC = 0;
            this.kB = 0.7;
            this.gamma = 1;
        }

        public PIDControlledDcMotorBuilder setkC(double kC) {
            this.kC = kC;
            return this;
        }

        public PIDControlledDcMotorBuilder setkB(double kB) {
            this.kB = kB;
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