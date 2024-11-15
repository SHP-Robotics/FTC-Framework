package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private final CRServo servo;

    public IntakeSubsystem(HardwareMap h){
        servo = (CRServo)h.get("intakeServo");
    }

    public CRServo getServo() {
        return servo;
    }


}
