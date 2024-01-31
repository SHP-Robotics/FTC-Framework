package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AccumulationControlledDcMotor extends DcMotorImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private final double MAX_VELOCITY = 12;

    private final AccumulationController padController;

    // gamma specifies de-acceleration multiplier vs acceleration (lower is faster de-acceleration)
    // check line 68
    private double gamma = 1;

    public AccumulationControlledDcMotor(AccumulationControlledDcMotorBuilder accumulationControlledDcMotorBuilder) {
        super(accumulationControlledDcMotorBuilder.dcMotor.getController(), accumulationControlledDcMotorBuilder.dcMotor.getPortNumber());
        super.setDirection(accumulationControlledDcMotorBuilder.dcMotor.getDirection());

        this.padController = new AccumulationController.AccumulationControllerBuilder(accumulationControlledDcMotorBuilder.kP)
                .setClampFunction(-1, 1)
                .build();

        this.gamma = accumulationControlledDcMotorBuilder.gamma;
    }

    public void setPower(double power, boolean decelerate) {
        double targetVelocity = MAX_VELOCITY*power;
        double currentVelocity = -this.getVelocity(AngleUnit.RADIANS);
        double error = targetVelocity - currentVelocity;

        double cGamma = decelerate ? gamma : 0;
        super.setPower(cGamma * padController.getOutput(error));
    }

    public static class AccumulationControlledDcMotorBuilder {
        private final DcMotor dcMotor;
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