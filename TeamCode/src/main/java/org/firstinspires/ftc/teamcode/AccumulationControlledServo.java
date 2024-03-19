package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AccumulationControlledServo extends ServoImplEx {
    private final double MAX_VELOCITY = 12;

    private final ElapsedTime elapsedTime;

    private double lastTime = -1;

    private double kP = 0.7;

    public AccumulationControlledServo(AccumulationControlledServoBuilder accumulationControlledServoBuilder) {
        super(
                (ServoControllerEx)accumulationControlledServoBuilder.servo.getController(),
                accumulationControlledServoBuilder.servo.getPortNumber(),
                ServoConfigurationType.getStandardServoType()
        );

        this.setDirection(accumulationControlledServoBuilder.servo.getDirection());

        this.kP = accumulationControlledServoBuilder.kP;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPosition(double error) {
        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;

        super.setPosition(clamp(getPosition() + (error * kP * deltaTime)));

        lastTime = time;
    }

    public static class AccumulationControlledServoBuilder {
        private final Servo servo;
        private double kP;

        public AccumulationControlledServoBuilder(Servo servo) {
            this.servo = servo;
            this.kP = 0.7;
        }

        public AccumulationControlledServoBuilder setkP(double kP) {
            this.kP = kP;
            return this;
        }

        public AccumulationControlledServo build() {
            return new AccumulationControlledServo(this);
        }
    }
}