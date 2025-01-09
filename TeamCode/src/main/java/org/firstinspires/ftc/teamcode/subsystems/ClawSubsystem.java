package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kClawName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kClose;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kOpen;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.State.OPEN;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class ClawSubsystem extends Subsystem {
    private final CachingServo claw;

    public enum State {
        OPEN,
        CLOSE,
        MANUAL;
    }
    private State state;

    public ClawSubsystem(HardwareMap hardwareMap){
        claw = new CachingServo((Servo) hardwareMap.get(kClawName));
        setState(State.CLOSE);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }
    public void increment(){
        state = State.MANUAL;
        claw.setPosition(claw.getPosition() + 0.01);
    }
    public void decrement(){
        state = State.MANUAL;
        claw.setPosition(claw.getPosition() - 0.01);
    }

    public void open(){
        state = State.OPEN;
        claw.setPosition(kOpen);
    }
    public void close(){
        state = State.CLOSE;
        claw.setPosition(kClose);
    }


    private void processState(State state) {
        if (this.state == State.CLOSE) {
            claw.setPosition(kClose);
        }
        else if (this.state == OPEN){
            claw.setPosition(kOpen);
        }
    }
    @Override
    public void periodic(Telemetry telemetry) {
        processState(state);

        telemetry.addData("Claw State: ", state);
        telemetry.addData("Claw Position: ", claw.getPosition());
    }


}
