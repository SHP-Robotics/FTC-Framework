package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.R;

public class Odometry extends DcMotorImplEx {
    private double TICKS_PER_INCH = 1;
    private int lastCall = 0;

    public Odometry(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
        this.setMode(RunMode.RUN_USING_ENCODER);
    }

    public Odometry(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
        this.setMode(RunMode.RUN_USING_ENCODER);
    }

    public Odometry(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
        this.setMode(RunMode.RUN_USING_ENCODER);
    }

    public Odometry(DcMotor dcMotor, double ticksPerInch) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        this.setDirection(dcMotor.getDirection());
        this.setMode(RunMode.RUN_USING_ENCODER);

        this.TICKS_PER_INCH = ticksPerInch;
    }

    public void reset() {
        this.lastCall = this.getCurrentPosition();
    }

//    Prevents front-end from controlling mode
//    @Override
//    public synchronized void setMode(RunMode mode) {}

    public double getInchesTravelled() {
        return (this.getCurrentPosition() - lastCall) / this.TICKS_PER_INCH;
    }
}
