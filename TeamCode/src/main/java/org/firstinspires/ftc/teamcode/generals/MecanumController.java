package org.firstinspires.ftc.teamcode.generals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumController {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private IMU imu;
    private float imuAngleOffset = 0;

    private float driveSpeed = 1;

    public void InitIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void Init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        InitIMU(hardwareMap);
    }

    private void InitDriverControlledTeleop() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void InitAutonomous() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void InitIntelligentTeleop() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private void InitIntelligentAutonomous() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public MecanumController(HardwareMap hardwareMap, RuntimeType runtimeType) {
        Init(hardwareMap);

        if (runtimeType == RuntimeType.DRIVER_CONTROLLED_TELEOP) {
            InitDriverControlledTeleop();
        } else if (runtimeType == RuntimeType.AUTONOMOUS) {
            InitAutonomous();
        } else if (runtimeType == RuntimeType.INTELLIGENT_TELEOP) {
            InitIntelligentTeleop();
        } else if (runtimeType == RuntimeType.INTELLIGENT_AUTONOMOUS) {
            InitIntelligentAutonomous();
        }
    }

    public void setDriveSpeed(float speed) {
        driveSpeed = speed;
    }

    public void calibrateIMUAngleOffset() {
        imuAngleOffset = -(float)imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public float getCalibratedIMUAngle() {
        return (float)imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + imuAngleOffset;
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
}
