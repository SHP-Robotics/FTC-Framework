package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kMaxHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kSlideTolerance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public SlideSubsystem(HardwareMap hardwareMap){
        slide = (DcMotorEx) hardwareMap.get("slide");
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPosition(double position){
        if(slide.getCurrentPosition() < kMaxHeight) {
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
}
