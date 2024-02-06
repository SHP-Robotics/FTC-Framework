package org.firstinspires.ftc.teamcode.debug;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AccumulationControlledServo extends ServoImplEx {
    // if too low, increasing kP will not speed up anymore, and may oscillate
    // should record the max velocity with kP = 1, and then use preferred values of kP
    private final double MAX_VELOCITY = 12;

    private ElapsedTime elapsedTime;

    private double lastError = 0;
    private double lastTime = -1;

    private double kP = 0.7;

    // gamma specifies de-acceleration multiplier vs acceleration (lower is faster de-acceleration)
    // check line 68
    private double gamma = 1;

    public AccumulationControlledServo(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType) {
        super(controller, portNumber, servoType);
    }

    public AccumulationControlledServo(AccumulationControlledServoBuilder accumulationControlledServoBuilder) {
        super(
                (ServoControllerEx)accumulationControlledServoBuilder.servo.getController(),
                accumulationControlledServoBuilder.servo.getPortNumber(),
                ServoConfigurationType.getStandardServoType()
        );

        this.setDirection(accumulationControlledServoBuilder.servo.getDirection());

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPosition(double error) {
        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;

        super.setPosition(clamp(getPosition() - (error * kP * deltaTime)));

        lastError = error;
        lastTime = time;
    }

    public static class AccumulationControlledServoBuilder {
        private Servo servo;
        private double kP;
        private double gamma;

        public AccumulationControlledServoBuilder(Servo servo) {
            this.servo = servo;
            this.kP = 0.7;
            this.gamma = 1;
        }

        public AccumulationControlledServoBuilder setkP(double kP) {
            this.kP = kP;
            return this;
        }

        public AccumulationControlledServoBuilder setGamma(double gamma) {
            this.gamma = gamma;
            return this;
        }

        public AccumulationControlledServo build() {
            return new AccumulationControlledServo(this);
        }
    }
}