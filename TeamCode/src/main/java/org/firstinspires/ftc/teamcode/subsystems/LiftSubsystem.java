package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class LiftSubsystem {

    public enum State {
        DOWN,

        DRIVE,

        DEPOSIT,

        MANUAL
    }

    public SHPMotor leftMotor;
    public SHPMotor rightMotor;

    public double power;

//    public Servo claw;

    public double staticPosition;
    public double targetPosition;

    private State state;



    public LiftSubsystem(HardwareMap hardwareMap, String leftMotor, String rightMotor) {
        this.leftMotor = new SHPMotor(hardwareMap, leftMotor, MotorUnit.TICKS);
        this.rightMotor = new SHPMotor(hardwareMap, rightMotor, MotorUnit.TICKS);

        this.leftMotor.resetEncoder();
        this.rightMotor.resetEncoder();

//        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.leftMotor.reverseDirection();


        this.leftMotor.enablePositionPID(new PositionPID(0.01, Constants.Lift.kLiftI, 0));
        this.rightMotor.enablePositionPID(new PositionPID(0.01, Constants.Lift.kLiftI, 0));



        this.leftMotor.setPositionErrorTolerance(5);
        this.rightMotor.setPositionErrorTolerance(5);


        this.state = State.MANUAL;
        targetPosition = 0;

        power = 1;
    }

    public String getState(){
        return state.toString();
    }

    public void setMotorDirection(boolean isRight, DcMotor.Direction direction) {
        if (isRight) {
            rightMotor.setDirection(direction);
        }
        else {
            leftMotor.setDirection(direction);
        }
    }

//    public void runToPosition(State state){
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        setPowerSynchronous(power);
//
//        switch (state){
//            case DOWN:
//                runToPosition(0);
//                break;
//            case DRIVE:
//                runToPosition(100);
//                break;
//
//            case DEPOSIT:
//                runToPosition(800);
//                break;
//        }
//
//    }

    public void setState(State state){
        this.state = state;
        switch (state){
            case DOWN:
                targetPosition = (int)Constants.Lift.kLiftDown;
                break;
            case DRIVE:
                targetPosition = (int)Constants.Lift.kLiftDrive;
                break;
            case DEPOSIT:
                targetPosition = (int)Constants.Lift.kLiftDeposit;
                break;
            case MANUAL:
               break;
        }

        leftMotor.setPosition(targetPosition);
        rightMotor.setPosition(targetPosition);

    }

//    public void runToPosition(int pos){
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setTargetPosition(pos);
//        rightMotor.setTargetPosition(pos);
//        setPowerSynchronous(power);
//    }

//    public void update( int tolerance) {
//        if (this.state == State.MANUAL) setPowerSynchronous(0);
//
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setTargetPosition((int)targetPosition);
//        rightMotor.setTargetPosition((int)targetPosition);
//
//
//        if (Math.abs(this.targetPosition-this.getAvgPos()) < tolerance){
////            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            applyBrakes();
//        }
//    }

    public void incrementState(){
        if (state == State.DOWN) setState(State.DRIVE);
        else if (state == State.DRIVE) setState(State.DEPOSIT);
        else if (state == State.MANUAL) setState(State.DEPOSIT);
    }

    public void deincrementState(){
        if (state == State.DEPOSIT) setState(State.DRIVE);
        else if (state == State.DRIVE) setState(State.DOWN);
        else if (state == State.MANUAL) setState(State.DEPOSIT);
    }

    public void applyBrakes() {
        staticPosition = this.getAvgPos();
//        runToPosition((int) staticPosition);
    }

    public void goUp() {
        setState(State.MANUAL);
        targetPosition = getAvgPos() + 30;
        rightMotor.setPosition(targetPosition);
        leftMotor.setPosition(targetPosition);
    }

    public void goDown() {
        setState(State.MANUAL);
        targetPosition = getAvgPos() - 30;
        rightMotor.setPosition(targetPosition);
        leftMotor.setPosition(targetPosition);
    }

    public double getAvgPos(){
        return (leftMotor.getPosition(MotorUnit.TICKS) + rightMotor.getPosition(MotorUnit.TICKS))/ 2.0;
    }

    public void setPowerSynchronous(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
