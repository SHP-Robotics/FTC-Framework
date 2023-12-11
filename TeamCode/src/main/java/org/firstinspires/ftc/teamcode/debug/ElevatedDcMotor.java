package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

public class ElevatedDcMotor extends DcMotorImpl {
    double weights = 1;
    double override = 0;
    boolean useOverride = false;

    public ElevatedDcMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public ElevatedDcMotor(DcMotor dcMotor) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        super.setDirection(dcMotor.getDirection());
    }

    public void setWeights(double weights) {
        this.weights = weights;
    }

    public double getWeights() {
        return this.weights;
    }

    public void setOverride(double override) {
        this.override = override;
    }

    public void setUseOverride(boolean useOverride) {
        this.useOverride = useOverride;
    }

    public double clamp(double power) {
        return Math.min(Math.max(-1, power), 1);
    }

    public void setPower(double power) {
        if (this.useOverride) {
            super.setPower(override);
        } else {
            super.setPower(clamp(power) * weights);
        }
    }
}
