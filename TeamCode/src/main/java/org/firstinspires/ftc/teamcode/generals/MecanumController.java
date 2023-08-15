package org.firstinspires.ftc.teamcode.generals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumController {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public IMU imu;
    private double imuAngleOffset = 0;

    private double driveSpeed = 1;
    private double rotationSpeed = 1;

    private double positionX = 0;
    private double positionY = 0;

    public void setMotorsRunMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        initIMU(hardwareMap);
    }

    private void initDriverControlledTeleop() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initAutonomous() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initIntelligentTeleop() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initIntelligentAutonomous() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public MecanumController(HardwareMap hardwareMap, RuntimeType runtimeType) {
        init(hardwareMap);

        if (runtimeType == RuntimeType.DRIVER_CONTROLLED_TELEOP) {
            initDriverControlledTeleop();
        } else if (runtimeType == RuntimeType.AUTONOMOUS) {
            initAutonomous();
        } else if (runtimeType == RuntimeType.INTELLIGENT_TELEOP) {
            initIntelligentTeleop();
        } else if (runtimeType == RuntimeType.INTELLIGENT_AUTONOMOUS) {
            initIntelligentAutonomous();
        }
    }

    public void setDriveSpeed(double speed) {
        driveSpeed = speed;
    }

    public void setRotationSpeed(double speed) {
        rotationSpeed = speed;
    }

    public void calibrateIMUAngleOffset() {
        imuAngleOffset = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getCalibratedIMUAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + imuAngleOffset;
    }

    public void drive(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_trigger - gamepad.left_trigger;

        double frontLeftPower = y + x + r;
        double frontRightPower = y - x - r;
        double backLeftPower = y - x + r;
        double backRightPower = y + x - r;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        frontLeft.setPower(frontLeftPower * driveSpeed / max);
        frontRight.setPower(frontRightPower * driveSpeed / max);
        backLeft.setPower(backLeftPower * driveSpeed / max);
        backRight.setPower(backRightPower * driveSpeed / max);
    }

    public void driverOrientedDrive(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_trigger - gamepad.left_trigger;

        double xOriented = x * Math.cos(getCalibratedIMUAngle()) - y * Math.sin(getCalibratedIMUAngle());
        double yOriented = x * Math.sin(getCalibratedIMUAngle()) + y * Math.cos(getCalibratedIMUAngle());

        double frontLeftPower = yOriented + xOriented + r;
        double frontRightPower = yOriented - xOriented - r;
        double backLeftPower = yOriented - xOriented + r;
        double backRightPower = yOriented + xOriented - r;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        frontLeft.setPower(frontLeftPower * driveSpeed / max);
        frontRight.setPower(frontRightPower * driveSpeed / max);
        backLeft.setPower(backLeftPower * driveSpeed / max);
        backRight.setPower(backRightPower * driveSpeed / max);
    }

    public boolean isBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }

    public void waitUntilCompletion() {
        while (isBusy()) {}
    }

    public void deactivate() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void rotateToRadian(double targetRadian, double radianTolerance) {
        setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetRadian = Constants.setToDomain(targetRadian, 0, 2 * Math.PI);
        double currentRadian = Constants.setToDomain(getCalibratedIMUAngle(), 0, 2 * Math.PI);
        while (Math.abs(currentRadian - targetRadian) > radianTolerance) {
            currentRadian = Constants.setToDomain(getCalibratedIMUAngle(), 0, 2 * Math.PI);
            double difference = targetRadian - currentRadian;
            if (difference > Math.PI) {
                difference -= 2 * Math.PI;
            } else if (difference < -Math.PI) {
                difference += 2 * Math.PI;
            }
            frontLeft.setPower(rotationSpeed);
            frontRight.setPower(-rotationSpeed);
            backLeft.setPower(rotationSpeed);
            backRight.setPower(-rotationSpeed);
        }
    }

    public void moveInches(double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches, boolean wait) {
        setMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double forward = (frontLeftInches + frontRightInches) / 2;
        double right = frontLeftInches - forward;

        double frontLeftEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD + right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;
        double frontRightEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD - right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;
        double backLeftEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD - right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;
        double backRightEncoderTicks = forward * Constants.WHEEL_ENCODER_TICKS_PER_INCH_FORWARD + right * Constants.WHEEL_ENCODER_TICKS_PER_INCH_SIDEWAYS;

        double max = Math.max(Math.max(Math.abs(frontLeftInches), Math.abs(frontRightInches)), Math.max(Math.abs(backLeftInches), Math.abs(backRightInches)));

        frontLeft.setTargetPosition((int) frontLeftEncoderTicks);
        frontRight.setTargetPosition((int) frontRightEncoderTicks);
        backLeft.setTargetPosition((int) backLeftEncoderTicks);
        backRight.setTargetPosition((int) backRightEncoderTicks);

        frontLeft.setPower(driveSpeed * frontLeftInches / max);
        frontRight.setPower(driveSpeed * frontRightInches / max);
        backLeft.setPower(driveSpeed * backLeftInches / max);
        backRight.setPower(driveSpeed * backRightInches / max);

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

        double frontLeftInches = yOriented + xOriented;
        double frontRightInches = yOriented - xOriented;
        double backLeftInches = yOriented - xOriented;
        double backRightInches = yOriented + xOriented;

        moveInches(frontLeftInches, frontRightInches, backLeftInches, backRightInches, wait);

        positionX = inchesX;
        positionY = inchesY;
    }
}
