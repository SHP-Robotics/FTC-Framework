package org.firstinspires.ftc.teamcode.shplib.controllers;

import com.qualcomm.robotcore.util.Range;

public class GainSchedule implements Comparable<GainSchedule> {
    public double kP, kI, kD, percentile = 0.0;

    public GainSchedule(double kP, double percentile) {
        this(kP, 0.0, 0.0, percentile);
    }

    public GainSchedule(double kP, double kI, double kD, double percentile) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.percentile = Range.clip(percentile, 0.0, 1.0);
    }

    public int compareTo(GainSchedule schedule) {
        return Double.compare(percentile, schedule.percentile);
    }
}
