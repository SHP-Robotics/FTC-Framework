package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kIntakeName;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private final CRServo intakeServo;
    public IntakeSubsystem(HardwareMap hardwareMap){
        intakeServo = (CRServo) hardwareMap.get(kIntakeName);
    }
    public CRServo getIntakeServo(){
        return intakeServo;
    }
    public void spinIn(){
        intakeServo.setPower(1.0);
    }
    public void spinOut(){
        intakeServo.setPower(-1.0);
    }

}
