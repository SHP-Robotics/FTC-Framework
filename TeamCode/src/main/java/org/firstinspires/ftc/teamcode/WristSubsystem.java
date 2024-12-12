package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.WristSubsystem.WristMode.OPEN;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystem {
    public enum WristMode {
        OPEN (0.5),
        CLOSE (1.0);


        WristMode(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }

        private final double position;
    }

    private Servo wrist;
    private WristMode mode= OPEN;
    //    private  final int offset=-1540;
    public WristSubsystem(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");

    }
    public void setOpen() {
        mode= OPEN;
    }
    public void setClose() {
        mode=WristMode.CLOSE;
    }
    public void update() {
        wrist.setPosition(mode.getPosition());
    }


    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Wrist Mode",mode);
        telemetry.addData("Wrist Position",wrist.getPosition());

    }
}
