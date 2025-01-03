package org.firstinspires.ftc.teamcode.subsystems;

//import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kCRWheelName;
//import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kSpinningIntakeName;
//import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kPixelServo;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class SensorSubsystem extends Subsystem {
    public final RevColorSensorV3 firstCS;
    public final RevColorSensorV3 secondCS;

    public enum State {
        IN,
        OUT
    }

    private State firstState;
    private State secondState;

    public SensorSubsystem(HardwareMap hardwareMap) {
       firstCS = hardwareMap.get(RevColorSensorV3.class, "firstCS");
       secondCS = hardwareMap.get(RevColorSensorV3.class, "firstCS");

       firstState = State.OUT;
       secondState = State.OUT;
    }
    public boolean filled(){
        if(firstState.equals(State.IN) && secondState.equals(State.IN)){
            return true;
        }
        return false;
    }
    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("FirstCS: ", firstState);
        telemetry.addData("SecondCS: ", secondState);

        if(firstCS.getDistance(DistanceUnit.INCH)<0.4){
            firstState = State.IN;
        }
        else{
            firstState = State.OUT;
        }

        if(secondCS.getDistance(DistanceUnit.INCH)<0.4){
            secondState = State.IN;
        }
        else{
            secondState = State.OUT;
        }


    }
}
