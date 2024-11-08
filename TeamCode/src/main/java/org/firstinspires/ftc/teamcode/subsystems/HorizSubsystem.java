package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kMaxHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kPixelHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kRightSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kSlideTolerance;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kLeftHorizSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRailName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRightHorizSlideName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class HorizSubsystem extends Subsystem {
    private final Servo lHoriz;
    private final Servo rHoriz;
    private final Servo rail;

    private int slidePos;

    public enum State {
        ALLIN(0, 0),
        HALFOUT(1,0),
        ALLOUT(1,1);

        final double railPos;
        final double slidePos;

        State(double railPos, double slidePos) {
            this.railPos = railPos;
            this.slidePos = slidePos;
        }
    }

    private State state;

    public HorizSubsystem(HardwareMap hardwareMap) {
        slidePos = 0;

        lHoriz = (Servo) hardwareMap.get(kLeftHorizSlideName);
        lHoriz.setDirection(Servo.Direction.FORWARD);

        rHoriz = (Servo) hardwareMap.get(kRightHorizSlideName);
        rHoriz.setDirection(Servo.Direction.REVERSE);

        rail = (Servo) hardwareMap.get(kRailName);
        rail.setDirection(Servo.Direction.REVERSE);

        setState(State.ALLIN);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }


//    public double getSlidePosition() {
//        return ((float)lHoriz.getCurrentPosition() + (float)rHoriz.getCurrentPosition()) / 2;
//    }

    public void nextState(){
        if(state == State.ALLIN)
            state = State.ALLOUT;
        else if (state == State.ALLOUT)
            state = State.HALFOUT;
        else
            state = State.ALLIN;
    }

    public void setSlidePos(int slidePos) {
        this.slidePos = slidePos;
    }

    public void incrementState(){
            slidePos += 10;
    }

    public void decrementState(){
            slidePos -= 10;
    }

    public void setPosition(double position){
        lHoriz.setPosition(position);
        rHoriz.setPosition(1.0-position);
    }

    private void processState() {
        if (this.state == State.ALLIN){
            setPosition(this.state.slidePos);
            rail.setPosition(this.state.railPos);
            return;
        }
        else if (this.state == State.ALLOUT) {
            setPosition(this.state.slidePos);
            rail.setPosition(this.state.railPos);
            return;
        }
        else if (this.state == State.HALFOUT) {
            setPosition(this.state.slidePos);
            rail.setPosition(this.state.railPos);
            return;
        }

        this.setPosition(this.state.position);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState();

        telemetry.addData("State: ", state);
        telemetry.addData("Left Slide Position: ", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Position: ", rightSlide.getCurrentPosition());
        telemetry.addData("Left current", leftSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right current", rightSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("avg current", (leftSlide.getCurrent(CurrentUnit.AMPS) + rightSlide.getCurrent(CurrentUnit.AMPS)) / 2);
//        telemetry.addData("Left Slide Velocity: ", leftSlide.getVelocity());
//        telemetry.addData("Right Slide Velocity: ", rightSlide.getVelocity());
    }
}
