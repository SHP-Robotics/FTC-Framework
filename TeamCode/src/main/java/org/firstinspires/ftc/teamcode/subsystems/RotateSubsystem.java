package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kNeutral;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kPickup;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kRotateName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class RotateSubsystem extends Subsystem {
    private final CachingServo rotate;
    private double rotatePos;

    public enum State {
        DROPOFF,
        INTAKE,
        PICKUP,
        AUTO,
        NEUTRAL;
    }
    private State state;

    public RotateSubsystem(HardwareMap hardwareMap){
        rotate = new CachingServo((Servo) hardwareMap.get(kRotateName));
        rotate.setDirection(Servo.Direction.FORWARD);
        rotatePos = kNeutral;
        setState(State.NEUTRAL);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void setPos(double pos){
        rotate.setPosition(pos);
    }
    public void rotateCW(){
        //if(state == State.INTAKE){
        state = State.INTAKE;
        rotatePos += 0.1;
        //}
    }
    public void rotateCCW() {
        //if (state == State.INTAKE) {
        state = State.INTAKE;
        rotatePos -= 0.1;
       // }
    }
    private void processState(State state) {
        if(this.state == State.INTAKE){
        }
        else if (this.state == State.NEUTRAL || this.state == State.DROPOFF) {
            rotatePos = kNeutral;
        }
        else if (this.state == State.PICKUP){
            rotatePos = kPickup;
        }
        else if (this.state == State.AUTO){
            rotatePos = 1;
        }
        rotate.setPosition(rotatePos);

    }
    @Override
    public void periodic(Telemetry telemetry) {
        processState(state);

        telemetry.addData("Rotate State: ", state);
        telemetry.addData("Rotate Position: ", rotate.getPosition());
    }


}
