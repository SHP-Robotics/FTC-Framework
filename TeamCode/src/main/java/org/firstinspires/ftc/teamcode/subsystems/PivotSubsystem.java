package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.kWristName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.klElbowName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.krElbowName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class PivotSubsystem extends Subsystem {
    private final IntakeSubsystem intake;
    private final Servo wrist;
    private final Servo lElbow;
    private final Servo rElbow;
    public PivotSubsystem(HardwareMap hardwareMap){
        intake = new IntakeSubsystem(hardwareMap);
        wrist = (Servo) hardwareMap.get(kWristName);
        lElbow = (Servo) hardwareMap.get(klElbowName);
        rElbow = (Servo) hardwareMap.get(krElbowName);
    }
    public void set4Bar(double pos){
        lElbow.setPosition(pos);
        rElbow.setPosition(1-pos);
    }
    public IntakeSubsystem getIntake(){
        return intake;
    }
    public Servo getWrist(){
        return wrist;
    }


}
