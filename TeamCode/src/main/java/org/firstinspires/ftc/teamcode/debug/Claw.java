package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo servo;
    double mostOpen = 0;
    double mostClose = 1;

    public Claw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "claw");
    }

    public Claw(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
    }

    public void setRange(double mostOpen, double mostClose) {
        this.mostOpen = mostOpen;
        this.mostClose = mostClose;
    }
}
