package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public abstract class AntiStallDcMotor extends DcMotorImplEx {
    private final double STALL_THRESHOLD;
    private double POWER_SCALAR = 1;

    public AntiStallDcMotor(DcMotorController controller, int portNumber, double STALL_THRESHOLD) {
        super(controller, portNumber);
        this.STALL_THRESHOLD = STALL_THRESHOLD;
    }

    public AntiStallDcMotor(DcMotor dcMotor, double STALL_THRESHOLD) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        this.STALL_THRESHOLD = STALL_THRESHOLD;
    }

    @Override
    public void setPower(double power) {
        super.setPower(power*this.POWER_SCALAR);
    }

    void reboot() {
        this.POWER_SCALAR = 1;
        this.setPower(this.getPower());
    }

    boolean isStalling() {
        return this.getCurrent(CurrentUnit.AMPS) >= this.STALL_THRESHOLD;
    }
}