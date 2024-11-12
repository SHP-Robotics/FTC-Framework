package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kIntakeName;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    }
    public void setState(State state){ this.state = state; }

    public State getState(){ return state; }
    public CRServo getIntakeServo(){
        return intakeServo;
    }
    public void intaking(double pow){
        intakeServo.setPower(pow);
    }
    public void runServo(){ intakeServo.setPower(state.power); }
    @Override
    public void periodic(Telemetry telemetry) {
//        runServo();
    }

}
