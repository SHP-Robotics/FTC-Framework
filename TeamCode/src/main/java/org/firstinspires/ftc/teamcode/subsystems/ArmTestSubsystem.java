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
    private final SHPMotor slide;
    private final SHPMotor rotate;
    private int slidePos;


    public ArmTestSubsystem(HardwareMap hardwareMap) {
        slidePos = 0;

        slide = new SHPMotor(hardwareMap, kSlideName);
                //(DcMotorEx) hardwareMap.get(kSlideName);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rotate = new SHPMotor(hardwareMap, kRotateName);
                //(DcMotorEx) hardwareMap.get(kRotateName);
        rotate.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void incrementSlide(){
        slide.setPosition(slide.getPosition(MotorUnit.TICKS)+10);
    }
    public void decrementSlide(){
        slide.setPosition(slide.getPosition(MotorUnit.TICKS)-10);
    }
    public void incrementRotate(){
        rotate.setPosition(rotate.getPosition(MotorUnit.TICKS)+10);
    }
    public void decrementRotate(){
        rotate.setPosition(rotate.getPosition(MotorUnit.TICKS)-10);
    }

    @Override
    public void periodic(Telemetry telemetry) {

        telemetry.addData("Slide Position: ", slide.getPosition(MotorUnit.TICKS));
        telemetry.addData("Rotate Position: ", rotate.getPosition(MotorUnit.TICKS));

    }
}
