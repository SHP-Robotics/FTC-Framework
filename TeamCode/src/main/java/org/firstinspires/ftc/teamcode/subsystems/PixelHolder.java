package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.kLeftAdjust;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kPositionBottom;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kPositionTop;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kRightAdjust;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class PixelHolder extends Subsystem {
    private final Servo adjustLeft;
    private final Servo adjustRight;
    private double clawPosition;
    public PixelHolder(HardwareMap hardwareMap) {
        adjustLeft = hardwareMap.get(Servo.class, kLeftAdjust);
        adjustRight = hardwareMap.get(Servo.class, kRightAdjust);
        clawPosition = kPositionBottom;
        descend();
    }

    public void descend() {
        adjustLeft.setPosition(kPositionBottom);
//        adjustRight.setPosition(1-kPositionBottom);
        clawPosition = kPositionBottom;
    }

    public void extend() {
        adjustRight.setPosition(kPositionTop);
        //if they are in different positions
//        adjustRight.setPosition(1-kPositionTop);
        clawPosition = kPositionTop;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Left Adjust Position: ", adjustLeft.getPosition());
        telemetry.addData("Right Adjust Position: ", adjustRight.getPosition());
    }
}
