package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.Intake.kIntakeName;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private final CRServo intakeServo;

    public enum State {
        INTAKING(1.0),
        OUTAKING(-1.0),
        PAUSED(0.0);

        final double power;

        State(double power){this.power = power;}
    }
    private State state;
    public IntakeSubsystem(HardwareMap hardwareMap){
        intakeServo = (CRServo) hardwareMap.get(kIntakeName);
       setState(State.PAUSED);
    }
    public void setState(State state){this.state = state;}
    public State getState(){return state;}
    public CRServo getIntakeServo(){
        return intakeServo;
    }
    public void runServo(){intakeServo.setPower(state.power);}

}
