package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Synchronous {
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public Synchronous(HardwareMap hardwareMap, String leftMotor, String rightMotor) {
        this.leftMotor = hardwareMap.get(DcMotor.class, leftMotor);
        this.rightMotor = hardwareMap.get(DcMotor.class, rightMotor);
    }

    public void setMotorDirection(Side side, DcMotor.Direction direction) {
        if (side == Side.LEFT) {
            leftMotor.setDirection(direction);
        }
        if (side == Side.RIGHT) {
            rightMotor.setDirection(direction);
        }
    }

    public void setPowerSynchronous(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
