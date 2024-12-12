package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ClawSubsystem.ClawMode.OPEN;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem {
    public enum ClawMode {
        OPEN (0.5),
        CLOSE (1.0);


        ClawMode(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }

        private final double position;
    }

    private Servo claw;
    private ClawMode mode= OPEN;
    //    private  final int offset=-1540;
    public ClawSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }
    public void setOpen() {
        mode= OPEN;
    }
    public void setClose() {
        mode=ClawMode.CLOSE;
    }
    public void update() {
        claw.setPosition(mode.getPosition());
    }


    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Claw Mode",mode);
        telemetry.addData("Claw Position",claw.getPosition());

    }
}
