package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kMaxHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kIncrement;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kRightSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kSlideTolerance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class VerticalSubsystem extends Subsystem {
    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private int slidePos;

    public enum State {
        BOTTOM(0),
        DEPOSITING(1000),
        DOWN(50),
        LOWBAR(0),
        HIGHBAR(650),
        LOWBUCKET(800), //TODO TUNE
        HIGHBUCKET(3000), //TODO TUNE
        MANUAL(0),
        NOPOWER(0);

        final double position;

        State(double position) {
            this.position = position;
        }
    }

    private State state, depositState;

    public VerticalSubsystem(HardwareMap hardwareMap) {
        slidePos = 0;

        leftSlide = (DcMotorEx) hardwareMap.get(kLeftSlideName);
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide = (DcMotorEx) hardwareMap.get(kRightSlideName);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        setState(State.MANUAL);
        depositState = State.HIGHBAR;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public double getDriveBias() {
        return 0.6;
    }

    public double getSlidePosition() {
        return ((float)leftSlide.getCurrentPosition() + (float)rightSlide.getCurrentPosition()) / 2;
    }

    public void incrementSlide(){
        if(slidePos <= kMaxHeight - kIncrement) {
            state = State.MANUAL;
            slidePos += kIncrement;
        }
    }

    public void decrementSlide(){
        if(slidePos >= kIncrement ) {
            state = State.MANUAL;
            slidePos -= kIncrement;
        }
    }
    public void resetZeroPosition(){ //TODO Switch to William's stalling detection 
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDepositState(State state){
        this.depositState = state;
    }
    public void cycleStates(boolean bar){
        if(bar) {
            if (depositState == State.HIGHBAR) {
                depositState = State.LOWBAR;
            } else {
                depositState = State.HIGHBAR;
            }
        }
        else{
            if (depositState == State.HIGHBUCKET) {
                depositState = State.LOWBUCKET;
            } else {
                depositState = State.HIGHBUCKET;
            }
        }
    }
    private void setPosition(double position){
        if(leftSlide.getCurrentPosition() < kMaxHeight) {
            rightSlide.setTargetPosition((int) position);
            leftSlide.setTargetPosition((int) position);

            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (Math.abs(this.getSlidePosition() - position) < kSlideTolerance) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
                return;
            }

            rightSlide.setPower(Constants.Vertical.kRunPower);
            leftSlide.setPower(Constants.Vertical.kRunPower);
        }
    }

    private void processState() {
        if (this.state == State.MANUAL) {
            this.setPosition(slidePos);
            return;
        }
        if (this.state == State.DEPOSITING){
            this.setPosition(this.depositState.position);
            return;
        }
        this.setPosition(this.state.position);

    }

    public void update(){
        periodic(telemetry);
    }
    @Override
    public void periodic(Telemetry telemetry) {
        processState();
        telemetry.addData("DEPOSIT STATE:", depositState);
        telemetry.addData("Slide State: ", state);
        telemetry.addData("Left Slide Position: ", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Position: ", rightSlide.getCurrentPosition());
    }
}
