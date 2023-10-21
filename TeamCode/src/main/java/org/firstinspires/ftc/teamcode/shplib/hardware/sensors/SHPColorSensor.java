package org.firstinspires.ftc.teamcode.shplib.hardware.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SHPColorSensor {
    private final ColorSensor sensor;

    public SHPColorSensor(HardwareMap hardwareMap, String deviceName) {
        sensor = hardwareMap.get(ColorSensor.class, deviceName);
    }

    public boolean isBlack() {
        return sensor.blue() > (3.0 / 4.0) * sensor.red();
    }

    public boolean isGreen() {
        return sensor.green() > sensor.blue() + sensor.red();
    }

    public boolean isBlue() {
        return sensor.blue() > sensor.green() + sensor.red();
    }

    public boolean isRed() {
        return sensor.red() > sensor.blue() + sensor.green();
    }
}
