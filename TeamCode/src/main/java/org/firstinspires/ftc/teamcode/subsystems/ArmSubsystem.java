package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowD;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowDown;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowDrive;
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

    // tee hee
//    private final SHPMotor elbow;
//    private final Servo wrist;
//    private double elbowManual, wristManual;
    public enum State {
        INTAKE,
        DRIVE,
        OUTTAKE,
        UNKNOWN,
        MANUAL,

        CLIMB
    }

    private State state;

    public ArmSubsystem(HardwareMap hardwareMap) {
//        poleSensor = hardwareMap.get(DistanceSensor.class, "coneSensor");

        // tee hee
//        elbow = new SHPMotor(hardwareMap, kElbowName);
//        elbow.reverseDirection();
//        elbow.resetEncoder();
//        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        elbow.enablePositionPID(new PositionPID(kElbowP, 0.0, kElbowD));
//        elbow.setPositionErrorTolerance(kElbowTolerance);
//        elbow.enableFF(new ElevatorFFController(kElbowS, kElbowG));

//        wrist = hardwareMap.get(Servo.class, kWristName);
//        wristManual = 0.5;
//        setState(State.DRIVE);
    }

    public void setState(State state) {
        this.state = state;
    }
    public State getState(){ return state;}

    // tee hee
//    public double getDriveBias() {
//        return Math.abs(getElbowPosition(MotorUnit.TICKS) / kElbowUp - 1.0);
//    }

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

    // tee hee
//    public double upElbow(){
//        elbowManual += 20;
//        return elbowManual;
//    }
//    public double downElbow(){
//        elbowManual -= 20;
//        return elbowManual;
//    }
//    public double upWrist(){
//        wristManual += 0.005;
//        return wristManual;
//    }
//    public double downWrist(){
//        wristManual -= 0.005;
//        return wristManual;
//    }
//    public boolean atSetpoint() {
//        return elbow.atPositionSetpoint();
//    }

//    public double getElbowPosition(MotorUnit unit) {
//        return elbow.getPosition(unit);
//    }

    private State processState() {
        // tee hee
//        switch (state) {
//            case DRIVE:
//                elbow.setPosition(kElbowDrive);
//                wrist.setPosition(kWristDrive);
//            case INTAKE:
//                elbow.setPosition(kElbowDown);
//                wrist.setPosition(kWristDown);
//            case OUTTAKE:
//                elbow.setPosition(kElbowUp);
//                wrist.setPosition(kWristDeposit);
//            case MANUAL:
//                elbow.setPosition(elbowManual);
//                wrist.setPosition(wristManual);
//        }
        return state;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState();
        // tee hee
//        telemetry.addData("Elbow Position: ", elbow.getPosition(MotorUnit.TICKS));
//        telemetry.addData("Wrist State: ", wrist.getPosition());
//        telemetry.addData("Wrist manual: ", wristManual);


//        telemetry.addData("ArmState: ", processState());
    }
}
