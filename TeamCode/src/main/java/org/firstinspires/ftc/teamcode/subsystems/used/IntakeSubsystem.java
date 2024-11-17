package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kIntakeName;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private final CRServo intakeServo;



    public IntakeSubsystem(HardwareMap hardwareMap){
        intakeServo = (CRServo) hardwareMap.get(kIntakeName);
    }

    public CRServo getIntakeServo(){
        return intakeServo;
    }
    public void setPower(double pow){
        intakeServo.setPower(pow);
    }
    @Override
    public void periodic(Telemetry telemetry) {
//        runServo();
    }

}
