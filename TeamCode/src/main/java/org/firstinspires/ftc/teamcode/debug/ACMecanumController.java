package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class ACMecanumController extends RobotController {
    public AccumulationControlledDcMotor leftFront;
    public AccumulationControlledDcMotor rightFront;
    public AccumulationControlledDcMotor leftRear;
    public AccumulationControlledDcMotor rightRear;

    public void setMotorsRunMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftRear.setZeroPowerBehavior(zeroPowerBehavior);
        rightRear.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public ACMecanumController(DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, IMU imu) {
        this.leftFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(leftFront)
                .setkP(1)
                .build();
        this.rightFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(rightFront)
                .setkP(1)
                .build();
        this.leftRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(leftRear)
                .setkP(1)
                .build();
        this.rightRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(rightRear)
                .setkP(1)
                .build();
    }

    public ACMecanumController(HardwareMap hardwareMap) {
        // TODO: Set name
        DcMotor leftFront = (DcMotor) hardwareMap.get("leftFront");
        DcMotor rightFront = (DcMotor) hardwareMap.get("rightFront");
        DcMotor leftRear = (DcMotor) hardwareMap.get("leftRear");
        DcMotor rightRear = (DcMotor) hardwareMap.get("rightRear");

        this.leftFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(leftFront)
                .setkP(0.2)
                .setGamma(0.2)
                .setMax(0.4)
                .build();
        this.rightFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(rightFront)
                .setkP(0.2)
                .setGamma(0.2)
                .setMax(0.4)
                .build();
        this.leftRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(leftRear)
                .setkP(0.2)
                .setGamma(0.2)
                .setMax(0.4)
                .build();
        this.rightRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(rightRear)
                .setkP(0.2)
                .setGamma(0.2)
                .setMax(0.4)
                .build();

        // TODO: Set direction
        this.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveParams(double x, double y, double r) {
        double leftFrontPower = y + x + r;
        double rightFrontPower = y - x - r;
        double leftRearPower = y - x + r;
        double rightRearPower = y + x - r;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    public void driveFieldParams(double x, double y, double r, double gyro) {
        // cos * y = how much right if gamepad forward
        // cos * x = how much right if gamepad right
        // sin * y = how much forward if gamepad forward
        // sin * x = how much forward if gamepad right
        double xOriented = (Math.sin(gyro) * x) + (Math.cos(gyro) * y);
        double yOriented = (Math.sin(gyro) * y) - (Math.cos(gyro) * x);

        double leftFrontPower = yOriented + xOriented + r;
        double rightFrontPower = yOriented - xOriented - r;
        double leftRearPower = yOriented - xOriented + r;
        double rightRearPower = yOriented + xOriented - r;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    public boolean isBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy();
    }

    public void waitUntilCompletion() {
        while (isBusy()) {}
    }

    public void deactivate() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}