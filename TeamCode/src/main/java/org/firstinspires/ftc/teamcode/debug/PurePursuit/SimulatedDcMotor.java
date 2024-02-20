package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.debug.AccumulationController;

public class SimulatedDcMotor extends DcMotorImplEx {
    boolean usingSimulation = false;
    public double lastVelocity;
    AccumulationController controller;

    public SimulatedDcMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
        this.controller = new AccumulationController.AccumulationControllerBuilder(0.01).build();
    }

    public SimulatedDcMotor(DcMotor dcMotor, double kP) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        super.setDirection(dcMotor.getDirection());
        this.controller = new AccumulationController.AccumulationControllerBuilder(kP).build();
    }

    public synchronized void setLastVelocity(double velocity) {
        this.usingSimulation = true;
        this.lastVelocity = velocity;
    }

    public double getLastOutput() {
        return this.controller.getLastOutput();
    }

    @Override
    public synchronized void setPower(double power) {
        if (usingSimulation && power != 0) {
            super.setPower(this.controller.getOutput(power - lastVelocity));
            return;
        }
        super.setPower(power);
    }

    @Override
    public synchronized double getVelocity() {
        if (usingSimulation) {
            return lastVelocity;
        }
        return super.getVelocity();
    }
}