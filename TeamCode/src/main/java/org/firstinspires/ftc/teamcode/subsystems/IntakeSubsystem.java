package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kCRWheelName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kSpinningIntakeName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kPixelServo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class IntakeSubsystem extends Subsystem {
    private final CRServo crWheel;
    private final Servo pixelServo;
    private final SHPMotor spinningIntake;

    public enum State {
        INTAKE,
        DEPOSIT1,
        DEPOSIT2,
        STILL,
        REJECT,
        REJECT_ALL,
        AUTO_INTAKE
    }

    private State state;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        crWheel = hardwareMap.get(CRServo.class, kCRWheelName);
        pixelServo = hardwareMap.get(Servo.class, kPixelServo);
        spinningIntake = new SHPMotor(hardwareMap, kSpinningIntakeName);
        spinningIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setState(State.STILL);
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

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Intake: ", state);

        switch (state) {
            case INTAKE:
                crWheel.setPower(-1.0);
                spinningIntake.setPower(1.0);
                pixelServo.setPosition(0.9);
                break;
            case AUTO_INTAKE:
                crWheel.setPower(-1.0);
                spinningIntake.setPower(0.5);
                pixelServo.setPosition(0.8);
                break;
            case DEPOSIT1:
                pixelServo.setPosition(0.5);
                break;
            case DEPOSIT2:
                crWheel.setPower(-1.0);
                pixelServo.setPosition(0.5);
                break;
            case REJECT:
                crWheel.setPower(0);
                spinningIntake.setPower(-0.8);
                break;
            case STILL:
                crWheel.setPower(0);
                spinningIntake.setPower(0.0);
                break;
            case REJECT_ALL:
                crWheel.setPower(1.0);
                spinningIntake.setPower(-0.8);
                break;
        }
    }
}
