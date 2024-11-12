package org.firstinspires.ftc.teamcode.subsystems.used;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kClawName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kClose;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kOpen;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ClawSubsystem extends Subsystem {
    private final Servo claw;
    private final double open;
    private final double close;
    public ClawSubsystem(HardwareMap hardwareMap){
        claw = (Servo) hardwareMap.get(kClawName);
        open = kOpen;
        close = kClose;
        close();
    }
    public void open(){
        claw.setPosition(open);
    }
    public void close(){
        claw.setPosition(close);
    }
    public void changeClaw(){
        if(claw.getPosition() == open){
            close();
        }
        else{
            open();
        }
    }

}
