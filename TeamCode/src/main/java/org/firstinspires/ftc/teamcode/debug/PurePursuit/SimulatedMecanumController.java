package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftRearDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightRearDirection;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.RobotController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

public class SimulatedMecanumController extends RobotController {
    public SimulatedDcMotor leftFront;
    public SimulatedDcMotor rightFront;
    public SimulatedDcMotor leftRear;
    public SimulatedDcMotor rightRear;

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

    public SimulatedMecanumController(HardwareMap hardwareMap) {
        // TODO: Set name
        this.leftFront = new SimulatedDcMotor((DcMotor) hardwareMap.get("leftFront"), 1);
        this.rightFront = new SimulatedDcMotor((DcMotor) hardwareMap.get("rightFront"), 1);
        this.leftRear = new SimulatedDcMotor((DcMotor) hardwareMap.get("leftRear"), 1);
        this.rightRear = new SimulatedDcMotor((DcMotor) hardwareMap.get("rightRear"), 1);

        // TODO: Set direction
        this.leftFront.setDirection(leftFrontDirection.inverted());
        this.rightFront.setDirection(rightFrontDirection.inverted());
        this.leftRear.setDirection(leftRearDirection.inverted());
        this.rightRear.setDirection(rightRearDirection);
    }

    public void driveParams(double x, double y, double r) {
        double leftFrontPower = y + x + r;
        double rightFrontPower = y - x - r;
        double leftRearPower = y - x + r;
        double rightRearPower = y + x - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        if (max < 1) {
            max = 1;
        }

        leftFront.setPower(leftFrontPower * Constants.leftFrontPower / max);
        rightFront.setPower(rightFrontPower * Constants.rightFrontPower / max);
        leftRear.setPower(leftRearPower * Constants.leftRearPower / max);
        rightRear.setPower(rightRearPower * Constants.rightRearPower / max);
    }

    public void driveFieldParams(double x, double y, double r, double gyro) {
        double xOriented = (Math.sin(gyro) * x) - (Math.cos(gyro) * y);
        double yOriented = (Math.sin(gyro) * y) + (Math.cos(gyro) * x);

        double leftFrontPower = yOriented + xOriented + r;
        double rightFrontPower = yOriented - xOriented - r;
        double leftRearPower = yOriented - xOriented + r;
        double rightRearPower = yOriented + xOriented - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        if (max < 1) {
            max = 1;
        }

        leftFront.setPower(leftFrontPower * Constants.leftFrontPower / max);
        rightFront.setPower(rightFrontPower * Constants.rightFrontPower / max);
        leftRear.setPower(leftRearPower * Constants.leftRearPower / max);
        rightRear.setPower(rightRearPower * Constants.rightRearPower / max);
    }

    public void simulateEncoders(Position2D deltaPosition) {
        double x = deltaPosition.getX();
        double y = deltaPosition.getY();
        double r = deltaPosition.getHeadingRadians() * Constants.MECANUM_WIDTH / 2;
        double realMax = Math.max(Math.abs(leftFront.getLastOutput()), Math.max(Math.abs(rightFront.getLastOutput()),
                Math.max(Math.abs(leftRear.getLastOutput()), Math.abs(rightRear.getLastOutput()))));

        if (realMax < 1) {
            realMax = 1;
        }

        leftFront.setLastVelocity((y + x + r) / realMax);
        rightFront.setLastVelocity((y - x - r) / realMax);
        leftRear.setLastVelocity((y - x + r) / realMax);
        rightRear.setLastVelocity((y + x - r) / realMax);
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