package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

public class ElevatedMecanumController extends MecanumController {
    public ElevatedMecanumController(HardwareMap hardwareMap) {
        super(hardwareMap);

        this.leftFront = new ElevatedDcMotor(leftFront);
        this.rightFront = new ElevatedDcMotor(rightFront);
        this.leftRear = new ElevatedDcMotor(leftRear);
        this.rightRear = new ElevatedDcMotor(rightRear);
    }

    public ElevatedMecanumController(HardwareMap hardwareMap, SpeedController speedController) {
        super(hardwareMap, speedController);

        this.leftFront = new ElevatedDcMotor(leftFront);
        this.rightFront = new ElevatedDcMotor(rightFront);
        this.leftRear = new ElevatedDcMotor(leftRear);
        this.rightRear = new ElevatedDcMotor(rightRear);
    }

    public void setWeights(double leftFrontWeights, double rightFrontWeights, double leftRearWeights, double rightRearWeights) {
        if (this.leftFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftFront).setWeights(leftFrontWeights);
        }

        if (this.rightFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightFront).setWeights(rightFrontWeights);
        }

        if (this.leftRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftRear).setWeights(leftRearWeights);
        }

        if (this.rightRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightRear).setWeights(rightRearWeights);
        }
    }

    public void fullThrottle() {
        this.leftFront.setPower(1);
        this.leftRear.setPower(1);
        this.rightFront.setPower(1);
        this.rightRear.setPower(1);
    }

    public void driveWeights(Gamepad gamepad) {
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);

        setWeights(y, y, y, y);
    }

    public void setOverride(double leftFrontOverride, double rightFrontOverride, double leftRearOverride, double rightRearOverride) {
        if (this.leftFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftFront).setOverride(leftFrontOverride);
        }

        if (this.rightFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightFront).setOverride(rightFrontOverride);
        }

        if (this.leftRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftRear).setOverride(leftRearOverride);
        }

        if (this.rightRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightRear).setOverride(rightRearOverride);
        }
    }

    public void activateOverride() {
        if (this.leftFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftFront).setUseOverride(true);
        }

        if (this.rightFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightFront).setUseOverride(true);
        }

        if (this.leftRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftRear).setUseOverride(true);
        }

        if (this.rightRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightRear).setUseOverride(true);
        }
    }

    public void disableOverride() {
        if (this.leftFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftFront).setUseOverride(false);
        }

        if (this.rightFront instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightFront).setUseOverride(false);
        }

        if (this.leftRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.leftRear).setUseOverride(false);
        }

        if (this.rightRear instanceof ElevatedDcMotor) {
            ((ElevatedDcMotor) this.rightRear).setUseOverride(false);
        }
    }

    public void driveOverride(Gamepad gamepad) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        double leftFrontPower = y + x + r;
        double rightFrontPower = y - x - r;
        double leftRearPower = y - x + r;
        double rightRearPower = y + x - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        speedController.updateSpeed(gamepad);
        this.setDriveSpeed(speedController.getSpeed());

        if (max < 1) {
            max = 1;
        }

        if (max * driveSpeed < Constants.MINIMUM_VOLTAGE_APPLIED) {
            leftFront.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            rightFront.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            leftRear.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            rightRear.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            return;
        }

        leftFrontPower *= driveSpeed / max;
        rightFrontPower *= driveSpeed / max;
        leftRearPower *= driveSpeed / max;
        rightRearPower *= driveSpeed / max;

        this.setOverride(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }
}