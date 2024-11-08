package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kMaxHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kPixelHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kRightSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kSlideTolerance;
import static org.firstinspires.ftc.teamcode.shplib.Constants.ArmTest.kRotateName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.ArmTest.kSlideName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class ArmTestSubsystem extends Subsystem {
    private final DcMotorEx slide;
    private final DcMotorEx rotate;
    private int slidePos;


    public ArmTestSubsystem(HardwareMap hardwareMap) {
        slidePos = 0;

        slide = (DcMotorEx) hardwareMap.get(kSlideName);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotate = (DcMotorEx) hardwareMap.get(kRotateName);
        rotate.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void incrementSlide(){
        slide.setTargetPosition(slide.getCurrentPosition()+10);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void decrementSlide(){

        slide.setTargetPosition(slide.getCurrentPosition()-10);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void incrementRotate(){

        rotate.setTargetPosition(rotate.getCurrentPosition()+10);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void decrementRotate(){

        rotate.setTargetPosition(rotate.getCurrentPosition()-10);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void periodic(Telemetry telemetry) {

        telemetry.addData("Slide Position: ", slide.getCurrentPosition());
        telemetry.addData("Rotate Position: ", rotate.getCurrentPosition());

    }
}
