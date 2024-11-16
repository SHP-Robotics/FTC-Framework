package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class WristSubsystem extends Subsystem {
    private Servo leftServo;
    private Servo rightServo;
    private double manualPos;
    public enum State {
        UP(0.3),
        MANUAL(0),
        DOWN(0.7);

        final double wristPos;

        State(double wristPos){
            this.wristPos = wristPos;
        }
    }
    private State state;

    public WristSubsystem(HardwareMap h){
        leftServo = (Servo)h.get("leftWrist");
        rightServo = (Servo)h.get("rightWrist");

        leftServo.setDirection(Servo.Direction.REVERSE);
        state = State.UP;
        manualPos = rightServo.getPosition();
    }
    public void setState(State state){
        this.state = state;
    }
    public State getState(){return state;}
    public void setWristPos(double pos){
        rightServo.setPosition(pos);
        leftServo.setPosition(1.0-pos);
    }
    public void incrementUp(){
        manualPos += 0.1;
    }
    public void incrementDown(){
        manualPos -= 0.1;
    }
    public void processState(){
        if(state == State.UP)
            setWristPos(State.UP.wristPos);
        else if(state == State.DOWN)
            setWristPos(State.DOWN.wristPos);
        else if (state == State.MANUAL){
            setWristPos(manualPos);
        }
    }
    @Override
    public void periodic(Telemetry telemetry){
        processState();
        telemetry.addData("Wrist Pos:", rightServo.getPosition());
        telemetry.update();
    }
}
