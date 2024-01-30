package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PADControlledDcMotor extends DcMotorImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private final double MAX_VELOCITY = 12;

    private final ElapsedTime elapsedTime;

    private double lastTime = -1;

    private final PADController padController;

    // gamma specifies de-acceleration multiplier vs acceleration (lower is faster de-acceleration)
    // check line 68
    private double gamma = 1;

    public PADControlledDcMotor(PADControlledDcMotorBuilder padControlledDcMotorBuilder) {
        super(padControlledDcMotorBuilder.dcMotor.getController(), padControlledDcMotorBuilder.dcMotor.getPortNumber());
        super.setDirection(padControlledDcMotorBuilder.dcMotor.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        this.padController = new PADController(
                padControlledDcMotorBuilder.kP,
                padControlledDcMotorBuilder.kD
        );

        this.gamma = padControlledDcMotorBuilder.gamma;
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

        double cGamma = decelerate ? gamma : 0;
        super.setPower(clamp(cGamma * padController.getOutput(error)));
        padController.setClampedLastOutput(super.getPower());

        lastTime = time;
    }

    public static class PADControlledDcMotorBuilder {
        private final DcMotor dcMotor;
        private double kP;
        private double kD;
        private double gamma;

        public PADControlledDcMotorBuilder(DcMotor dcMotor) {
            this.dcMotor = dcMotor;
            this.kP = 0.7;
            this.kD = 0;
            this.gamma = 1;
        }

        public PADControlledDcMotorBuilder setkP(double kP) {
            this.kP = kP;
            return this;
        }

        public PADControlledDcMotorBuilder setkD(double kD) {
            this.kD = kD;
            return this;
        }

        public PADControlledDcMotorBuilder setGamma(double gamma) {
            this.gamma = gamma;
            return this;
        }

        public PADControlledDcMotor build() {
            return new PADControlledDcMotor(this);
        }
    }
}