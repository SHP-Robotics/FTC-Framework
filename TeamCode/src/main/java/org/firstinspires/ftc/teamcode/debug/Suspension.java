package org.firstinspires.ftc.teamcode.generals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Suspension {
    private DcMotor suspensionLeft;
    private DcMotor suspensionRight;

    public Suspension(HardwareMap hardwareMap) {
        suspensionLeft = hardwareMap.get(DcMotor.class, "suspensionLeft");
        suspensionRight = hardwareMap.get(DcMotor.class, "suspensionRight");
    }

    public Suspension(DcMotor suspensionLeft, DcMotor suspensionRight) {
        this.suspensionLeft = suspensionLeft;
        this.suspensionRight = suspensionRight;
    }

    public void setPower(double power) {
        suspensionLeft.setPower(power);
        suspensionRight.setPower(power);
    }
}
