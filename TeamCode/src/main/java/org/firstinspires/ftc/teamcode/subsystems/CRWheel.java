package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kCRWheelName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWheelBackward;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWheelForward;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kWheelStill;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
public class CRWheel extends Subsystem {
    // Declare devices
    // Example:
    private final CRServo cWheel;
    //private final Servo pixelThing; // Need better name
    public enum State {
        FORWARD, BACKWARD, STILL
//        STILL, PIXELON, PIXELOFF
    }

    private State state;

    public CRWheel(HardwareMap hardwareMap) {
        cWheel = hardwareMap.get(CRServo.class, kCRWheelName);
//        pixelThing = hardwareMap.get(Servo.class, kPixelThingName);
        // Set initial state
        // Example:
        setState(State.STILL);
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        switch (state) {
            case FORWARD:
                cWheel.setPower(kWheelForward);
                break;
            case BACKWARD:
                cWheel.setPower(kWheelBackward);
                break;
            case STILL:
                cWheel.setPower(kWheelStill);
                break;
        }
    }
}
