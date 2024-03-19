package org.firstinspires.ftc.teamcode.shplib.controllers;

public class ElevatorFFController extends FFController {
    private final double kG;

    public ElevatorFFController(double kS, double kG) {
        super(kS);
        this.kG = kG;
    }

    public ElevatorFFController(double kV, double kS, double kG) {
        super(kV, kS);
        this.kG = kG;
    }

    public ElevatorFFController(double kV, double kA, double kS, double kG) {
        super(kV, kA, kS);
        this.kG = kG;
    }

    @Override
    public double getStaticOutput(double power) {
        return super.getStaticOutput(power) + kG;
    }
}
