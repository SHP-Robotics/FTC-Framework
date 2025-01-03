package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Sensors.kClawColorName;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class DetectionSubsystem extends Subsystem {
    private final ColorSensor clawColor;

    public DetectionSubsystem(HardwareMap hardwareMap){
        clawColor = (ColorSensor) hardwareMap.get(kClawColorName);

    }
    public String getMaxColor(){
        if(clawColor.red() > clawColor.green() && clawColor.red() > clawColor.blue())
            return "r";
        else if(clawColor.blue() > clawColor.red() && clawColor.blue() > clawColor.green())
            return "b";
        else
            return "g";
    }
    public int getVal(String color){
        switch (color) {
            case "r":
                return clawColor.red();
            case "g":
                return clawColor.green();
            case "b":
                return clawColor.blue();
            default:
                return -1;
        }
    }

    @Override
    public void periodic(Telemetry telemetry){
        telemetry.addData("Claw Color Red: ", clawColor.red());
        telemetry.addData("Claw Color Green: ", clawColor.green());
        telemetry.addData("Claw Color Blue: ", clawColor.blue());
        telemetry.addData("Claw Color Alpha: ", clawColor.alpha());
    }




}
