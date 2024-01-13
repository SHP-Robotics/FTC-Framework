package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowD;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowDown;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowG;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowName;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowP;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowS;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowTolerance;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowUp;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kWristDeposit;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kWristDown;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kWristDrive;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kWristName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.controllers.ElevatorFFController;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class ArmSubsystem extends Subsystem {

    private final SHPMotor elbow;
    private final Servo wrist;
    private int elbowManual, wristManual;
    public enum State {
        INTAKE,
        DRIVE,
        OUTTAKE,
        UNKNOWN,
        MANUAL
    }

    private State state;

    public ArmSubsystem(HardwareMap hardwareMap) {
//        poleSensor = hardwareMap.get(DistanceSensor.class, "coneSensor");

        elbow = new SHPMotor(hardwareMap, kElbowName);
        //leftSlide.reverseDirection();
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elbow.enablePositionPID(new PositionPID(kElbowP, 0.0, kElbowD));
        elbow.setPositionErrorTolerance(kElbowTolerance);
        elbow.enableFF(new ElevatorFFController(kElbowS, kElbowG));

        wrist = hardwareMap.get(Servo.class, kWristName);

        setState(State.DRIVE);
    }

    public void setState(State state) {
        this.state = state;
    }
    public State getState(){ return state;}
    public double getDriveBias() {
        return Math.abs(getElbowPosition(MotorUnit.TICKS) / kElbowUp - 1.0);
    }

//    public boolean isOverPole() {
//        return poleSensor.getDistance(DistanceUnit.INCH) <= 6.0;
//    }

//    public boolean atStacks() {
//        return state == State.STACK;
//    }
//
//    public boolean atHub() {
//        return state == State.HUB;
//    }
    public double setElbow(){
        elbowManual += 10;
        return elbowManual;
    }
    public double setWrist(){
        wristManual += 0.05;
        return wristManual;
    }
    public boolean atSetpoint() {
        return elbow.atPositionSetpoint();
    }

    public double getElbowPosition(MotorUnit unit) {
        return elbow.getPosition(unit);
    }

    private State processState() {
        switch (state) {
            case DRIVE:
                elbow.setPosition(kElbowDown);
                wrist.setPosition(kWristDrive);
                return state;
            case INTAKE:
                elbow.setPosition(kElbowDown);
                wrist.setPosition(kWristDown);
                return state;
            case OUTTAKE:
                elbow.setPosition(kElbowUp);
                wrist.setPosition(kWristDeposit);
                return state;
            case MANUAL:
                elbow.setPosition(elbowManual);
                wrist.setPosition(wristManual);
        }
        return State.UNKNOWN;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Elbow Position: ", elbow.getPosition(MotorUnit.TICKS));
        telemetry.addData("ArmState: ", processState());
    }
}
