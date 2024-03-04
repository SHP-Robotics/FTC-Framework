package org.firstinspires.ftc.teamcode.debug;

import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftRearDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightRearDirection;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

public class MecanumController implements RobotController {
    public DcMotorEx[] motors;

    public IMU imu;
    protected double imuAngleOffset = 0;

    protected SpeedController speedController;
    protected double driveSpeed = 1;
    protected double rotationSpeed = 1;

    // setRotationSpeed affects rotationKP also
    private double rotationKP = 1/Math.PI;

    private double positionX = 0;
    private double positionY = 0;

    public void setMotorsRunMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor: motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public MecanumController(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftRear, DcMotorEx rightRear, IMU imu) {
        this.motors = new DcMotorEx[]{
                leftFront,
                rightFront,
                leftRear,
                rightRear
        };

        this.imu = imu;

        this.speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .build();
    }

    public MecanumController(HardwareMap hardwareMap) {
        // TODO: Set name
        this.motors = new DcMotorEx[]{
                (DcMotorEx) hardwareMap.get("leftFront"),
                (DcMotorEx) hardwareMap.get("rightFront"),
                (DcMotorEx) hardwareMap.get("leftRear"),
                (DcMotorEx) hardwareMap.get("rightRear")
        };

        // TODO: Set direction
        this.motors[0].setDirection(leftFrontDirection);
        this.motors[1].setDirection(rightFrontDirection);
        this.motors[2].setDirection(leftRearDirection);
        this.motors[3].setDirection(rightRearDirection);

        // TODO: Set name
        this.imu = (IMU) hardwareMap.get("imu");
        // TODO: Set orientation
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));

        this.speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
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
        imuAngleOffset = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+Math.PI/2;
//        imu.resetYaw();
    }

    public double getCalibratedIMUAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + imuAngleOffset;
    }

    public void driveParams(double x, double y, double r, double[] bottleneckSpeeds) {
        double[] powers = new double[]{
                y + x + r,
                y - x - r,
                y - x + r,
                y + x - r
        };

        double max = 0;
        for (double power: powers) {
            if (Math.abs(power) > max) {
                max = Math.abs(power);
            }
        }

        if (max < 1) {
            max = 1;
        }

        for (int i = 0; i < 4; i++) {
            this.motors[i].setPower(powers[i] * Constants.powers[i] * driveSpeed * bottleneckSpeeds[i] / max);
        }
    }

    public void driveParams(double x, double y, double r) {
        this.driveParams(x, y, r, new double[]{1, 1, 1, 1});
    }

    public void drive(Gamepad gamepad) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        speedController.updateSpeed(gamepad);
        this.driveSpeed = speedController.getSpeed();

        this.driveParams(x, y, r);
    }

    public void driveFieldParams(double x, double y, double r, double gyro, double[] bottleneckSpeeds) {
        double xOriented = (Math.sin(gyro) * x) - (Math.cos(gyro) * y);
        double yOriented = (Math.sin(gyro) * y) + (Math.cos(gyro) * x);

        this.driveParams(xOriented, yOriented, r, bottleneckSpeeds);
    }

    public void driveFieldParams(double x, double y, double r, double gyro) {
        this.driveFieldParams(x, y, r, gyro, new double[]{1, 1, 1, 1});
    }

    public void fieldOrientedDrive(Gamepad gamepad) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        speedController.updateSpeed(gamepad);
        this.driveSpeed = speedController.getSpeed();

        this.driveFieldParams(x, y, r, this.getCalibratedIMUAngle());
    }

    public void fieldOrientedDrive(Gamepad gamepad, double[] bottleneckSpeeds) {
        double x = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_RIGHT);
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);
        double r = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.ROTATE_RIGHT);

        speedController.updateSpeed(gamepad);
        this.driveSpeed = speedController.getSpeed();

        this.driveFieldParams(x, y, r, this.getCalibratedIMUAngle(), bottleneckSpeeds);
    }

    public boolean isBusy() {
        for (DcMotorEx motor: this.motors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    public void waitUntilCompletion() {
        while (isBusy()) {}
    }

    public void deactivate() {
        for (DcMotorEx motor: this.motors) {
            motor.setPower(0);
        }
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

            this.motors[0].setPower(-direction * this.rotationSpeed);
            this.motors[1].setPower(direction * this.rotationSpeed);
            this.motors[2].setPower(-direction * this.rotationSpeed);
            this.motors[3].setPower(direction * this.rotationSpeed);

            currentRadians = this.getCalibratedIMUAngle();
            relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);
        }

        this.deactivate();
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

            this.motors[0].setPower(power);
            this.motors[1].setPower(-power);
            this.motors[2].setPower(power);
            this.motors[3].setPower(-power);

            currentRadians = this.getCalibratedIMUAngle();
            relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);
        }

        this.deactivate();
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

            this.motors[0].setPower(power);
            this.motors[1].setPower(-power);
            this.motors[2].setPower(power);
            this.motors[3].setPower(-power);

            currentRadians = this.getCalibratedIMUAngle();
            relativeRadians = Constants.setToDomain(currentRadians, targetRadian - Math.PI, targetRadian + Math.PI);
        }

        this.deactivate();
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

        this.motors[0].setTargetPosition((int) leftFrontEncoderTicks);
        this.motors[1].setTargetPosition((int) rightFrontEncoderTicks);
        this.motors[2].setTargetPosition((int) leftRearEncoderTicks);
        this.motors[3].setTargetPosition((int) rightRearEncoderTicks);

        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.motors[0].setPower(driveSpeed * leftFrontInches / max);
        this.motors[1].setPower(driveSpeed * rightFrontInches / max);
        this.motors[2].setPower(driveSpeed * leftRearInches / max);
        this.motors[3].setPower(driveSpeed * rightRearInches / max);

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