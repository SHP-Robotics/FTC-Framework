package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kIncrement;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kMaxHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kSlideTolerance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class SlideSubsystem extends Subsystem {
    private DcMotorEx slide;

    public enum State{
        ALLIN(0),
        LOWOUT(500),
        MIDOUT(1000),
        HIGHOUT(2100),
        INCREMENTING(0);
        final double slidePos;
        State(double slidePos){this.slidePos = slidePos;}
    }
    private State state;
    private double manualPos;
    public SlideSubsystem(HardwareMap hardwareMap){
        slide = (DcMotorEx) hardwareMap.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manualPos = slide.getCurrentPosition();

        state = State.ALLIN;
    }
    public void setState(State state){this.state = state;}
    public DcMotorEx getSlide(){return slide;}
    public void setPosition(double position) {
        if (slide.getCurrentPosition() < kMaxHeight) {
            slide.setTargetPosition((int) position);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (Math.abs(slide.getCurrentPosition() - position) < kSlideTolerance) {
                slide.setPower(0);
                return;
            }

//        if (this.state == State.BOTTOM) {
//            if (this.getSlidePosition() < kSlideTolerance) {
//                rightSlide.setPower(0);
//                leftSlide.setPower(0);
//                return;
//            }
//        }

            slide.setPower(1);
        }
    }
    public void incrementUp(){
        if(slide.getCurrentPosition() <= kMaxHeight-kIncrement) {
            this.state = State.INCREMENTING;
            manualPos += kIncrement;
        }
    }
    public void incrementDown(){
        if(slide.getCurrentPosition() >= kIncrement){
            this.state = State.INCREMENTING;
            manualPos -= kIncrement;
        }

    }
    public void processState(){
        if (manualPos < 10){
            this.slide.setPower(0);
            return;
        }

        if (this.state == State.INCREMENTING) {
            this.setPosition(manualPos);

        }
        else
            this.setPosition(this.state.slidePos);
    }

    public void setPower(double power){
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setPower(power);
    }
    @Override
    public void periodic(Telemetry telemetry){
        processState();
//        telemetry.addData("Slide Pos:", slide.getCurrentPosition());
//        telemetry.addData("Power:", slide.getPower());
//        telemetry.addData("State:", state);
//        telemetry.addData("Manual Val:", manualPos);
//        telemetry.addData("Slide Mode", slide.getMode());
//        telemetry.addData("Target Pos:", slide.getTargetPosition());
//        telemetry.update();

    }
}
