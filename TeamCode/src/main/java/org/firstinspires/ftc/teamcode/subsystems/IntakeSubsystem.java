package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private final CRServo servo;

    public enum State{
        INTAKING(1.0),
        PAUSED(0),
        OUTTAKING(-1.0);
        final double power;
        State(double power){this.power = power;}
    }
    private State state;
    public IntakeSubsystem(HardwareMap h){

        servo = (CRServo)h.get("intakeServo");
        state = State.PAUSED;
    }

    public CRServo getServo() {
        return servo;
    }
    public void processState(){
        servo.setPower(this.state.power);
    }


}
