package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftSubsystem {

    public enum State {
        DOWN,

        DRIVE,

        DEPOSIT
    }

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public double power;

//    public Servo claw;

    public double staticPosition;



    public LiftSubsystem(HardwareMap hardwareMap, String leftMotor, String rightMotor) {
        this.leftMotor = hardwareMap.get(DcMotor.class, leftMotor);
        this.rightMotor = hardwareMap.get(DcMotor.class, rightMotor);

        power = 1;
    }

    public void setMotorDirection(boolean isRight, DcMotor.Direction direction) {
        if (isRight) {
            rightMotor.setDirection(direction);
        }
        else {
            leftMotor.setDirection(direction);
        }
    }

    public void runToPosition(State state){
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerSynchronous(power);

        switch (state){
            case DOWN:
                runToPosition(0);
                break;
            case DRIVE:
                runToPosition(100);
                break;

            case DEPOSIT:
                runToPosition(800);
                break;
        }

    }

    public void runToPosition(int pos){
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(pos);
        rightMotor.setTargetPosition(pos);
        setPowerSynchronous(power);
    }

    public void update( int tolerance) {
        if (Math.abs(leftMotor.getTargetPosition()-this.getAvgPos()) < tolerance){
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            applyBrakes();
        }
    }

    public void applyBrakes() {
        staticPosition = this.getAvgPos();
        runToPosition((int) staticPosition);
    }

    public double getAvgPos(){
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/ 2.0;
    }

    public void setPowerSynchronous(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
