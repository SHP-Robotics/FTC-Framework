package org.firstinspires.ftc.teamcode.shplib.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

// TODO: make schedules sorted by percentile to optimize calculate method

public class ScheduledPIDController extends PIDController {
    private final ArrayList<GainSchedule> schedules;
    private double initialPosition = 0.0;
    private int index = 0;

    public ScheduledPIDController(double kP) {
        this(kP, 0.0, 0.0);
    }

    public ScheduledPIDController(double kP, double kI, double kD) {
        super(kP, kI, kD);
        this.schedules = new ArrayList<>();
        scheduleGains(new GainSchedule(kP, kI, kD, 1.0));
    }

    public ScheduledPIDController(double kP, double kI, double kD, GainSchedule... schedules) {
        this(kP, kI, kD);
        scheduleGains(schedules);
    }

    public void scheduleGains(GainSchedule... schedules) {
        this.schedules.addAll(Arrays.asList(schedules));
        Collections.sort(this.schedules);
    }

    public void setInitialPosition(double initialPosition) {
        this.initialPosition = initialPosition;
    }

    public double getInitialPosition() {
        return initialPosition;
    }

    @Override
    public double calculate(double current, double setpoint) {
        return calculate(initialPosition, current, setpoint);
    }

//    public double calculate(double initial, double current, double setpoint) {
//        double percentComplete = getPercentComplete(initial, current, setpoint);
//
//        // find lowest percentile that is greater than percentComplete
//        double lowest = 1.0;
//        for (GainSchedule schedule : schedules) {
//            if (schedule.percentile >= percentComplete &&
//                    schedule.percentile <= lowest) {
//                setGains(schedule.kP, schedule.kI, schedule.kD);
//                lowest = schedule.percentile;
//            }
//        }
//
//        return super.calculate(current, setpoint);
//    }

    public double calculate(double initial, double current, double setpoint) {
        double percentComplete = getPercentComplete(initial, current, setpoint);
        double percentile = schedules.get(index).percentile;
        if (percentComplete > percentile) nextSchedule();
        scheduleGains(schedules.get(index));
        return super.calculate(current, setpoint);
    }

    public void nextSchedule() {
        if (index != schedules.size() - 1) index++;
    }

    public void setScheduleIndex(int index) {
        this.index = index;
    }

    public int getScheduleIndex() {
        return index;
    }

    public double getPercentComplete(double initial, double current, double setpoint) {
        return Math.abs(current - initial) / Math.abs(setpoint - initial);
    }

    // temporary method for testing
    public void logGains(Telemetry telemetry) {
        telemetry.addData("==========", "=");
        telemetry.addData("kP: ", kP);
        telemetry.addData("kI: ", kI);
        telemetry.addData("kD: ", kD);
    }
}
