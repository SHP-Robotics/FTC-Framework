package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControlledDcMotor extends DcMotorImplEx {
    private double kP;
    private double kD;

    private double lastError = 0;
    private double lastTime = 0;

    private ElapsedTime elapsedTime;

    public PIDControlledDcMotor(PIDControlledDcMotorBuilder pidControlledDcMotorBuilder) {
        super(pidControlledDcMotorBuilder.dcMotor.getController(), pidControlledDcMotorBuilder.dcMotor.getPortNumber());
        super.setDirection(pidControlledDcMotorBuilder.dcMotor.getDirection());

        this.kP = pidControlledDcMotorBuilder.kP;
        this.kD = pidControlledDcMotorBuilder.kD;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPower(double error) {
        double time = elapsedTime.seconds();
        double dError = lastError - error;
        double dTime = lastTime - time;

        super.setPower(clamp((kP * error) + (kD * dError/dTime)));

        lastError = error;
        lastTime = time;
    }

    public static class PIDControlledDcMotorBuilder {
        private final DcMotor dcMotor;
        private double kP;
        private double kD;

        public PIDControlledDcMotorBuilder(DcMotor dcMotor) {
            this.dcMotor = dcMotor;
            this.kP = 0.7;
            this.kD = 0.01;
        }

        public PIDControlledDcMotorBuilder setkP(double kP) {
            this.kP = kP;
            return this;
        }

        public PIDControlledDcMotorBuilder setkD(double kD) {
            this.kD = kD;
            return this;
        }

        public PIDControlledDcMotor build() {
            return new PIDControlledDcMotor(this);
        }
    }
}