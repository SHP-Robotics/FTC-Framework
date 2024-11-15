package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class WristSubsystem extends Subsystem {
    private Servo leftServo;
    private Servo rightServo;

    public enum State {
        UP(0.3),
        DOWN(0.7);

        final double wristPos;

        State(double wristPos){
            this.wristPos = wristPos;
        }
    }

    public WristSubsystem(HardwareMap h){
        leftServo = (Servo)h.get("leftWrist");
        rightServo = (Servo)h.get("rightWrist");

        leftServo.setDirection(Servo.Direction.REVERSE);
    }
}
