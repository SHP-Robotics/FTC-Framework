package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.debug.config.*;

public class MecanumController {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;

    public IMU imu;
    protected double imuAngleOffset = 0;

    protected SpeedController speedController;
    protected double driveSpeed = 1;
    protected double rotationSpeed = 1;

    // setRotationSpeed affects rotationKP also
    private double rotationKP = 1/Math.PI;

    private double positionX = 0;
    private double positionY = 0;

    public static int sign(float num) {
        if (num == 0) {
            return 0;
        }
        return (int)(Math.abs(num)/num);
    }

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

    public MecanumController(DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, IMU imu) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;

        this.imu = imu;

        this.speedController = new SpeedController.SpeedBuilder(SpeedType.NO_CHANGE)
                .build();
    }

    public MecanumController(HardwareMap hardwareMap) {
        // TODO: Set name
        this.leftFront = (DcMotor) hardwareMap.get("leftFront");
        this.rightFront = (DcMotor) hardwareMap.get("rightFront");
        this.leftRear = (DcMotor) hardwareMap.get("leftRear");
        this.rightRear = (DcMotor) hardwareMap.get("rightRear");

        // TODO: Set direction
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightRear.setDirection(DcMotor.Direction.FORWARD);

        // TODO: Set name
        this.imu = (IMU) hardwareMap.get("imu");
        // TODO: Set orientation
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));

        this.speedController = new SpeedController.SpeedBuilder(SpeedType.NO_CHANGE)
                .build();
    }

    public void fakeReset() {
        this.positionX = 0;
        this.positionY = 0;
    }

    public void setSpeedController(SpeedController speedController) {
        this.speedController = speedController;
    }

    public double getDriveSpeed() {
        return this.driveSpeed;
    }

    public void setDriveSpeed(double speed) {
        this.speedController.setSpeed(speed);
        this.driveSpeed = speed;
    }

    public void setRotationSpeed(double rotationSpeed) {
        this.rotationSpeed = rotationSpeed;
        this.rotationKP = rotationSpeed / Math.PI;
    }

    public void calibrateIMUAngleOffset() {
        imuAngleOffset = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        imu.resetYaw();
    }

    public double getCalibratedIMUAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + imuAngleOffset;
    }

    public void drive(Gamepad gamepad) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        double leftFrontPower = y + x + r;
        double rightFrontPower = y - x - r;
        double leftRearPower = y - x + r;
        double rightRearPower = y + x - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        speedController.updateSpeed(gamepad, max);
        this.driveSpeed = speedController.getSpeed();

        if (max < 1) {
            max = 1;
        }

        leftFront.setPower(leftFrontPower * driveSpeed / max);
        rightFront.setPower(rightFrontPower * driveSpeed / max);
        leftRear.setPower(leftRearPower * driveSpeed / max);
        rightRear.setPower(rightRearPower * driveSpeed / max);
    }

    public void drivePID(Gamepad gamepad) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        double leftFrontPower = y + x + r;
        double rightFrontPower = y - x - r;
        double leftRearPower = y - x + r;
        double rightRearPower = y + x - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        speedController.updateSpeed(gamepad, max);
        this.driveSpeed = speedController.getSpeed();

        if (max < 1) {
            max = 1;
        }

        boolean deceleration = x == 0 && y == 0 && r == 0;

        if (this.leftFront instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) leftFront).setPower(leftFrontPower * driveSpeed / max, deceleration);
        } else {
            leftFront.setPower(leftFrontPower * driveSpeed / max);
        }

        if (this.rightFront instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) rightFront).setPower(rightFrontPower * driveSpeed / max, deceleration);
        } else {
            rightFront.setPower(leftFrontPower * driveSpeed / max);
        }

        if (this.leftRear instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) leftRear).setPower(leftRearPower * driveSpeed / max, deceleration);
        } else {
            leftRear.setPower(leftFrontPower * driveSpeed / max);
        }

        if (this.rightRear instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) rightRear).setPower(rightRearPower * driveSpeed / max, deceleration);
        } else {
            rightRear.setPower(leftFrontPower * driveSpeed / max);
        }
    }

    public void fieldOrientedDrive(Gamepad gamepad) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        // cos * y = how much right if gamepad forward
        // cos * x = how much right if gamepad right
        // sin * y = how much forward if gamepad forward
        // sin * x = how much forward if gamepad right
        double xOriented = (Math.cos(getCalibratedIMUAngle()) * x) + (Math.sin(getCalibratedIMUAngle()) * y);
        double yOriented = (Math.cos(getCalibratedIMUAngle()) * y) - (Math.sin(getCalibratedIMUAngle()) * x);

        double leftFrontPower = yOriented + xOriented + r;
        double rightFrontPower = yOriented - xOriented - r;
        double leftRearPower = yOriented - xOriented + r;
        double rightRearPower = yOriented + xOriented - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        speedController.updateSpeed(gamepad, max);
        this.driveSpeed = speedController.getSpeed();

        if (max < 1) {
            max = 1;
        }

        leftFront.setPower(leftFrontPower * driveSpeed / max);
        rightFront.setPower(rightFrontPower * driveSpeed / max);
        leftRear.setPower(leftRearPower * driveSpeed / max);
        rightRear.setPower(rightRearPower * driveSpeed / max);
    }

    public void fieldOrientedDrivePID(Gamepad gamepad) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        // cos * y = how much right if gamepad forward
        // cos * x = how much right if gamepad right
        // sin * y = how much forward if gamepad forward
        // sin * x = how much forward if gamepad right
        double xOriented = (Math.cos(getCalibratedIMUAngle()) * x) + (Math.sin(getCalibratedIMUAngle()) * y);
        double yOriented = (Math.cos(getCalibratedIMUAngle()) * y) - (Math.sin(getCalibratedIMUAngle()) * x);

        double leftFrontPower = yOriented + xOriented + r;
        double rightFrontPower = yOriented - xOriented - r;
        double leftRearPower = yOriented - xOriented + r;
        double rightRearPower = yOriented + xOriented - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        speedController.updateSpeed(gamepad, max);
        this.driveSpeed = speedController.getSpeed();

        if (max < 1) {
            max = 1;
        }

        boolean deceleration = xOriented == 0 && yOriented == 0 && r == 0;

        if (this.leftFront instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) leftFront).setPower(leftFrontPower * driveSpeed / max, deceleration);
        } else {
            leftFront.setPower(leftFrontPower * driveSpeed / max);
        }

        if (this.rightFront instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) rightFront).setPower(rightFrontPower * driveSpeed / max, deceleration);
        } else {
            rightFront.setPower(leftFrontPower * driveSpeed / max);
        }

        if (this.leftRear instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) leftRear).setPower(leftRearPower * driveSpeed / max, deceleration);
        } else {
            leftRear.setPower(leftFrontPower * driveSpeed / max);
        }

        if (this.rightRear instanceof AccumulationControlledDcMotor) {
            ((AccumulationControlledDcMotor) rightRear).setPower(rightRearPower * driveSpeed / max, deceleration);
        } else {
            rightRear.setPower(leftFrontPower * driveSpeed / max);
        }
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

        if (speedController.applyMinimumVoltage && max * driveSpeed < Constants.MINIMUM_VOLTAGE_APPLIED) {
            leftFront.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            rightFront.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            leftRear.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            rightRear.setPower(Constants.MINIMUM_VOLTAGE_APPLIED);
            return;
        }

        leftFront.setPower(leftFrontPower * driveSpeed / max);
        rightFront.setPower(rightFrontPower * driveSpeed / max);
        leftRear.setPower(leftRearPower * driveSpeed / max);
        rightRear.setPower(rightRearPower * driveSpeed / max);
    }

    public void driveFieldParams(double x, double y, double r, double gyro) {
        // cos * y = how much right if gamepad forward
        // cos * x = how much right if gamepad right
        // sin * y = how much forward if gamepad forward
        // sin * x = how much forward if gamepad right
        double xOriented = (Math.cos(gyro) * x) + (Math.sin(gyro) * y);
        double yOriented = (Math.cos(gyro) * y) - (Math.sin(gyro) * x);

        double leftFrontPower = yOriented + xOriented + r;
        double rightFrontPower = yOriented - xOriented - r;
        double leftRearPower = yOriented - xOriented + r;
        double rightRearPower = yOriented + xOriented - r;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        if (max < 1) {
            max = 1;
        }

        leftFront.setPower(leftFrontPower * driveSpeed / max);
        rightFront.setPower(rightFrontPower * driveSpeed / max);
        leftRear.setPower(leftRearPower * driveSpeed / max);
        rightRear.setPower(rightRearPower * driveSpeed / max);
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

    public void rotateToRadian(double targetRadian, double radianTolerance) {
        this.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentRadians = this.getCalibratedIMUAngle();
        double relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);
        double direction;
        double dif;

        while (!((relativeRadians < targetRadian && (relativeRadians + radianTolerance) > targetRadian)
                || (relativeRadians > targetRadian && (relativeRadians - radianTolerance) < targetRadian))) {
            dif = targetRadian - relativeRadians;
            direction = dif / (Math.abs(dif));

            this.leftFront.setPower(-direction * this.rotationSpeed);
            this.rightFront.setPower(direction * this.rotationSpeed);
            this.leftRear.setPower(-direction * this.rotationSpeed);
            this.rightRear.setPower(direction * this.rotationSpeed);

            currentRadians = this.getCalibratedIMUAngle();
            relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);
        }

        this.leftFront.setPower(0);
        this.rightFront.setPower(0);
        this.leftRear.setPower(0);
        this.rightRear.setPower(0);
    }

    public void rotateToRadianUsingPID(double targetRadian, double radianTolerance) {
        this.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentRadians = this.getCalibratedIMUAngle();
        double relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);

        double error = targetRadian - relativeRadians;

        double power;

        while (!(error > -radianTolerance && error < radianTolerance)) {
            error = targetRadian - relativeRadians;

            power = error * this.rotationKP;

            if (power < 0) {
                power = Math.max(Constants.MINIMUM_VOLTAGE_APPLIED, Math.min(1, power));
            } else {
                power = Math.min(-Constants.MINIMUM_VOLTAGE_APPLIED, Math.max(-1, power));
            }

            this.leftFront.setPower(power);
            this.rightFront.setPower(-power);
            this.leftRear.setPower(power);
            this.rightRear.setPower(-power);

            currentRadians = this.getCalibratedIMUAngle();
            relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);
        }

        this.leftFront.setPower(0);
        this.rightFront.setPower(0);
        this.leftRear.setPower(0);
        this.rightRear.setPower(0);
    }

    public void rotateToRadianUsingPID(double targetRadian, double radianTolerance, int maxOscillations) {
        this.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentRadians = this.getCalibratedIMUAngle();
        double relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);

        double lastError = 0.0;
        double error = targetRadian - relativeRadians;

        double power;

        double numOscillations = 0;

        while (numOscillations < maxOscillations && !(error > -radianTolerance && error < radianTolerance)) {
            error = targetRadian - relativeRadians;

            if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) {
                numOscillations += 1;
            }

            power = error * this.rotationKP;

            if (power < 0) {
                power = Math.max(Constants.MINIMUM_VOLTAGE_APPLIED, Math.min(1, power));
            } else {
                power = Math.min(-Constants.MINIMUM_VOLTAGE_APPLIED, Math.max(-1, power));
            }

            lastError = error;

            this.leftFront.setPower(power);
            this.rightFront.setPower(-power);
            this.leftRear.setPower(power);
            this.rightRear.setPower(-power);

            currentRadians = this.getCalibratedIMUAngle();
            relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);
        }

        this.leftFront.setPower(0);
        this.rightFront.setPower(0);
        this.leftRear.setPower(0);
        this.rightRear.setPower(0);
    }

    public void moveInches(double leftFrontInches, double rightFrontInches, double leftRearInches, double rightRearInches, boolean wait) {
        setMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double forward = (leftFrontInches + rightFrontInches) / 2;
        double right = leftFrontInches - forward;

        double leftFrontEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD + right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;
        double rightFrontEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD - right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;
        double leftRearEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD - right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;
        double rightRearEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD + right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;

        double max = Math.max(Math.max(Math.abs(leftFrontInches), Math.abs(rightFrontInches)), Math.max(Math.abs(leftRearInches), Math.abs(rightRearInches)));

        leftFront.setTargetPosition((int) leftFrontEncoderTicks);
        rightFront.setTargetPosition((int) rightFrontEncoderTicks);
        leftRear.setTargetPosition((int) leftRearEncoderTicks);
        rightRear.setTargetPosition((int) rightRearEncoderTicks);

        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(driveSpeed * leftFrontInches / max);
        rightFront.setPower(driveSpeed * rightFrontInches / max);
        leftRear.setPower(driveSpeed * leftRearInches / max);
        rightRear.setPower(driveSpeed * rightRearInches / max);

        if (wait) {
            waitUntilCompletion();
            deactivate();
        }
    }

    public void moveToPosition(double inchesX, double inchesY, boolean wait) {
        double x = inchesX - positionX;
        double y = inchesY - positionY;

        double xOriented = x * Math.cos(getCalibratedIMUAngle()) - y * Math.sin(getCalibratedIMUAngle());
        double yOriented = x * Math.sin(getCalibratedIMUAngle()) + y * Math.cos(getCalibratedIMUAngle());

        double leftFrontInches = yOriented + xOriented;
        double rightFrontInches = yOriented - xOriented;
        double leftRearInches = yOriented - xOriented;
        double rightRearInches = yOriented + xOriented;

        moveInches(leftFrontInches, rightFrontInches, leftRearInches, rightRearInches, wait);

        positionX = inchesX;
        positionY = inchesY;
    }
}