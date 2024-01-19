package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.kCRWheelName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kPixelServo;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kSpinningIntakeName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kWheelBackward;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kWheelForward;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kWheelStill;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class IntakeSubsystem extends Subsystem {
    // Declare devices
    // Example:
    private final CRServo cWheel;
    private final Servo pixelServo;
    private final SHPMotor spinner;

    public final Servo dropDown;
    //private final Servo pixelThing; // Need better name
    public enum State {
        INTAKING, OUTTAKING, STILL, REJECT,OUTAKE1, OUTNOWHEEL
//        PIXELIN, PIXELOUT
//        STILL, PIXELON, PIXELOFF
    }

    private State state;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        cWheel = hardwareMap.get(CRServo.class, kCRWheelName);
        pixelServo = hardwareMap.get(Servo.class, kPixelServo);
        spinner = new SHPMotor(hardwareMap, kSpinningIntakeName);
        dropDown = hardwareMap.get(Servo.class, "dropDown");


//        pixelThing = hardwareMap.get(Servo.class, kPixelThingName);
        // Set initial state
        // Example:
        setState(State.STILL);
        dropDown.setPosition(0.5);
        pixelServo.setPosition(0.9);
    }

    public void putPixelServoBack(){
        pixelServo.setPosition(0.9);
    }

    public boolean firstPixel(){
        return (pixelServo.getPosition()==0.5);
    }
    public void setState(State state) {
        this.state = state;
    }

    public State getState(){return state;}



    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }

    @Override
    public void periodic(Telemetry telemetry) {
        // Add logging if needed
        // Example:
         telemetry.addData("Intake: ", state);
        //dropDown.setPosition(0.5);
        switch (state) {
            case INTAKING:
                cWheel.setPower(-1.0);
                spinner.setPower(1);
                pixelServo.setPosition(0.9);
                break;
            case OUTTAKING:
                cWheel.setPower(-1.0);
                pixelServo.setPosition(0.5);
                break;
            case OUTNOWHEEL:
                spinner.setPower(-1);
                break;
            case STILL:
                cWheel.setPower(0);
                spinner.setPower(0.0);
                break;
            case REJECT:
                cWheel.setPower(1.0);
                spinner.setPower(-1);
                break;
            case OUTAKE1:
                pixelServo.setPosition(0.5);
//                pixelServo.setPosition(1.0);
                break;
//            case PIXELOUT:
////                pixelServo.setPosition(0.5);
//                break;

//            case PIXELOFF:
//                pixelThing.setPosition(kPixelDisengaged);
//            case PIXELON:
//                pixelThing.setPosition(kPixelEngaged);


        }
//        }

        // OR

//        if (state == State.ENABLED) {
//            setPower(1.0);
//        } else if (state == State.DISABLED) {
//            setPower(0.0);
//        }
    }
}
