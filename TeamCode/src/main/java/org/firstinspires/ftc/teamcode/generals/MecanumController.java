package org.firstinspires.ftc.teamcode.generals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumController {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private float driveSpeed = 1;

    public void Init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
    }

    public void InitDriverControlledTeleop() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void InitAutonomous() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void InitIntelligentTeleop() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void InitIntelligentAutonomous() {
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
}
