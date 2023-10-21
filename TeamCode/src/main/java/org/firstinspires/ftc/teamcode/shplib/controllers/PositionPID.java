package org.firstinspires.ftc.teamcode.shplib.controllers;

public class PositionPID extends ScheduledPIDController {
    private double currentPosition = 0.0;

    public PositionPID(double kP) {
        super(kP);
    }

    public PositionPID(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public double calculate(double desiredPosition) {
        return super.calculate(currentPosition, desiredPosition);
    }

    /**
     * Only use if you want to provide a different currentPosition
     *
     * @param currentPosition
     * @param desiredPosition
     * @return
     */
    @Override
    public double calculate(double currentPosition, double desiredPosition) {
        return super.calculate(currentPosition, desiredPosition);
    }
}
