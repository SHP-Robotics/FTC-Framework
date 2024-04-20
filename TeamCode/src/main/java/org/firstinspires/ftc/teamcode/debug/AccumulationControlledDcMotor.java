package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

public class AccumulationControlledDcMotor extends DcMotorImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private boolean usingSimulation = false;
    private double currentVelocity;

    private final ElapsedTime elapsedTime;

    private double lastTime = -1;
    private double lastError = -1;

    private final double kP;
    private final double kD;
    private final double gamma;

    public AccumulationControlledDcMotor(AccumulationControlledDcMotorBuilder accumulationControlledDcMotorBuilder) {
        super(accumulationControlledDcMotorBuilder.dcMotor.getController(), accumulationControlledDcMotorBuilder.dcMotor.getPortNumber());
        super.setDirection(accumulationControlledDcMotorBuilder.dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        this.kP = accumulationControlledDcMotorBuilder.kP;
        this.kD = accumulationControlledDcMotorBuilder.kD;
        this.gamma = accumulationControlledDcMotorBuilder.gamma;
    }

    private static double clamp(double x) {
        return Math.max(-1, Math.min(1, x));
    }

    @Override
    public void setPower(double power) {
        double targetVelocity = Constants.MAX_VELOCITY*power;
        double currentVelocity = this.getVelocity(AngleUnit.RADIANS);
        double error = targetVelocity - currentVelocity;
        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;
        double deltaError = error - lastError;

        if (targetVelocity == 0) {
            super.setPower(0);
            return;
        }

        super.setPower(clamp(getPower() + (error * kP * deltaTime) + (deltaError * kD * deltaTime)));

        lastError = error;
        lastTime = time;
    }

    public static class AccumulationControlledDcMotorBuilder {
        private final DcMotor dcMotor;
        private double kP;
        private double kD;
        private double gamma;

        public AccumulationControlledDcMotorBuilder(DcMotor dcMotor) {
            this.dcMotor = dcMotor;
            this.kP = 0.7;
            this.kD = 0;
            this.gamma = 0.5;
        }

        public AccumulationControlledDcMotorBuilder setkP(double kP) {
            this.kP = kP;
            return this;
        }

        public AccumulationControlledDcMotorBuilder setkD(double kD) {
            this.kD = kD;
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