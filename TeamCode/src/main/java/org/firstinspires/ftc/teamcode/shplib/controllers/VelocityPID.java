package org.firstinspires.ftc.teamcode.shplib.controllers;

import org.firstinspires.ftc.teamcode.Constants;

public class VelocityPID extends PIDController {
    private double currentVelocity = 0.0;
    private double power = 0.0;

    public VelocityPID(double kP) {
        super(kP);
        setCurrentVelocity(currentVelocity);
    }

    public VelocityPID(double kP, double kI, double kD) {
        super(kP, kI, kD);
        setCurrentVelocity(currentVelocity);
    }

    public void setCurrentVelocity(double currentVelocity) {
        this.currentVelocity = currentVelocity;
    }

    public double calculate(double desiredVelocity) {
        power += super.calculate(currentVelocity, desiredVelocity);
        return power;
    }

    /**
     * Only use if you want to provide a different currentVelocity
     *
     * @param currentVelocity
     * @param desiredVelocity
     * @return
     */
    @Override
    public double calculate(double currentVelocity, double desiredVelocity) {
        power += super.calculate(currentVelocity, desiredVelocity);
        return power;
    }
}
