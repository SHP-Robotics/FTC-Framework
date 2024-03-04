package org.firstinspires.ftc.teamcode.debug;

import java.util.ArrayList;

public final class PowerManager {
    private ArrayList<Object> consumers = new ArrayList<Object>();
    private static final double MAX_DRAWABLE_CURRENT = 0;

    public void update() {
        for (Object consumer: consumers) {
            if (consumer instanceof AntiStallDcMotor) {
                if (((AntiStallDcMotor)consumer).isStalling()) {
                    ((AntiStallDcMotor)consumer).reboot();
                }
            }
        }
    }

    public void addConsumer(Object consumer) {
        consumers.add(consumer);
    }

    public void clear() {
        consumers = new ArrayList<Object>();
    }
}
