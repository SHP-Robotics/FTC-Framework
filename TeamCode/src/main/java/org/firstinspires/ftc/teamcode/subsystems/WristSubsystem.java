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
        INTAKING(0.98),
        OUTTAKING(0.6),
        DRIVING(0.9);


        final double wristPos;

        State(double wristPos){
            this.wristPos = wristPos;
        }
    }
    private State state;

    public WristSubsystem(HardwareMap h){
        leftServo = (Servo)h.get("leftWrist");
        rightServo = (Servo)h.get("rightWrist");
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);
        state = State.INTAKING;
        manualPos = rightServo.getPosition();
    }
    public void setState(State state){
        this.state = state;
    }
    public State getState(){return state;}
    public void setWristPos(double pos){
        leftServo.setPosition(pos);
        //leftServo.setPosition(1.0-pos);
    }
    public void incrementUp(){
        manualPos += 0.1;
    }
    public void incrementDown(){
        manualPos -= 0.1;
    }
    public void processState(){
        setWristPos(this.state.wristPos);
    }
    @Override
    public void periodic(Telemetry telemetry){
        processState();
        telemetry.addData("Wrist Pos:", rightServo.getPosition());
        telemetry.update();
    }
}
