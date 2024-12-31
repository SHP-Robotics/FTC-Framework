package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.kWristName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.klElbowName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.krElbowName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kNeutral;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kPickup;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kRotateName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class RotateSubsystem extends Subsystem {
    private final Servo rotate;
    private double rotatePos;

    public enum State {
        DROPOFF,
        INTAKE,
        PICKUP,
        NEUTRAL;
    }
    private State state;

    public RotateSubsystem(HardwareMap hardwareMap){
        rotate = (Servo) hardwareMap.get(kRotateName);
        rotate.setDirection(Servo.Direction.FORWARD);
        rotatePos = 0;
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
        rotatePos = rotate.getPosition() + 0.005;
        //}
    }
    public void rotateCCW() {
        //if (state == State.INTAKE) {
        state = State.INTAKE;
        rotatePos = rotate.getPosition() - 0.005;
       // }
    }
    private void processState(State state) {
        if(this.state == State.INTAKE){
            rotate.setPosition(rotatePos + kNeutral);
        }
        if (this.state == State.NEUTRAL || this.state == State.DROPOFF) {
            rotatePos = 0;
            rotate.setPosition(kNeutral);
        }
        else if (this.state == State.PICKUP){
            rotatePos = 0;
            rotate.setPosition(kPickup);
        }
    }
    @Override
    public void periodic(Telemetry telemetry) {
        processState(state);

        telemetry.addData("Rotate State: ", state);
        telemetry.addData("Rotate Position: ", rotate.getPosition());
    }


}
