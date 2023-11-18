package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

public class WeightedDcMotor extends DcMotorImpl {
    double weights = 1;

    public WeightedDcMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public WeightedDcMotor(DcMotor dcMotor) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        super.setDirection(dcMotor.getDirection());
    }

    public void setWeights(double weights) {
        this.weights = weights;
    }

    public double getWeights() {
        return this.weights;
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPower(double power) {
        super.setPower(clamp(power) * weights);
    }
}
