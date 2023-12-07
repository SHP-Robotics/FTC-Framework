package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Plane.kHexagonName;
import static org.firstinspires.ftc.teamcode.Constants.Plane.kMissileLauncherName;
import static org.firstinspires.ftc.teamcode.Constants.Plane.kPlaneName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class PlaneSubsystem extends Subsystem {
    // Declare devices
    // Example:
     private final Servo plane;
     private final Servo missileLauncher;
     private final Servo hexagon;

    public enum State {
        // Define states
        // Example:
        // ENABLED, DISABLED
    }

    private State state;

    public PlaneSubsystem(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        plane = hardwareMap.get(Servo.class, kPlaneName);
        hexagon = hardwareMap.get(Servo.class, kHexagonName);
        missileLauncher = hardwareMap.get(Servo.class, kMissileLauncherName);

        // Set initial state
        // Example:
        // setState(State.TOP);
    }

    public void setState(State state) {
        this.state = state;
    }

    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }

    public void resetPlane(){
        plane.setPosition(0.5);
    }
    public void releasePlane(){
        plane.setPosition(0.0);
    }
    public void deactivteMissile(){
        missileLauncher.setPosition(0.17);
    }
    public void prepareMissile(){
        missileLauncher.setPosition(0.2175);//0.5 is halfway
    }


    public void resetHexagon(){
        hexagon.setPosition(0.8);
    }
    public void dropHexagon(){
        hexagon.setPosition(0.3);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        // Add logging if needed
        // Example:
        // telemetry.addData("Motor Encoder: ", motor.getPosition(MotorUnit.TICKS));

        // Handle states
        // Example:
//        switch (state) {
//            case ENABLED:
//                setPower(1.0);
//                break;
//            case DISABLED:
//                setPower(0.0);
//                break;
//        }

        // OR

//        if (state == State.ENABLED) {
//            setPower(1.0);
//        } else if (state == State.DISABLED) {
//            setPower(0.0);
//        }
    }
}
