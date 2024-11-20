package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class SparkMiniMotor extends DcMotorImplEx {
    CRServo servo;

    public SparkMiniMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public SparkMiniMotor(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public SparkMiniMotor(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }

    public void setServo(CRServo servo) {
        this.servo = servo;
    }

    @Override
    public synchronized void setPower(double power) {
        byte direction = 1;
        if (this.direction == Direction.REVERSE) direction = -1;
        this.servo.setPower(power * direction);
    }

    @Override
    public synchronized double getPower() {
        byte direction = 1;
        if (this.direction == Direction.REVERSE) direction = -1;
        return this.servo.getPower()*direction;
    }
}
