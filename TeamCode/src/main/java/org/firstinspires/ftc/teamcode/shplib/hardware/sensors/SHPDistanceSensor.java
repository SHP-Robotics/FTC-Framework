package org.firstinspires.ftc.teamcode.shplib.hardware.sensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.filters.Filter;

public class SHPDistanceSensor {
    private final DistanceSensor sensor;
    private final DistanceUnit unit;
    private final double offset;

    private Filter filter;

    public SHPDistanceSensor(@NonNull HardwareMap hardwareMap, String deviceName, DistanceUnit unit) {
        this(hardwareMap, deviceName, unit, 0.0);
    }

    public SHPDistanceSensor(@NonNull HardwareMap hardwareMap, String deviceName, DistanceUnit unit, double offset) {
        this.sensor = hardwareMap.get(DistanceSensor.class, deviceName);
        this.unit = unit;
        this.offset = offset;
    }

    public SHPDistanceSensor addFilter(Filter filter) {
        this.filter = filter;
        return this;
    }

    public double getDistance() {
        double distance = sensor.getDistance(unit);
        return (filter == null ? distance : filter.calculate(distance)) + offset;
    }
}
