package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;

@Config
public class PestoFTCConfig {
    public static double ODOMETRY_TICKS_PER_INCH = 336.877962878;
    public static double FORWARD_OFFSET = -1.4666181267717;
    public static double ODOMETRY_WIDTH = 9.0886313275906;
    public static double DECELERATION = -2398 / 28.9;
    public static double MAX_VELOCITY = 76;

    public static String frontLeftWheelName = "frontLeft";
    public static String frontRightWheelName = "frontRight";
    public static String backLeftWheelName = "backLeft";
    public static String backRightWheelName = "backRight";

    public static DcMotorSimple.Direction frontLeftWheelDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction frontRightWheelDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction backLeftWheelDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction backRightWheelDirection = DcMotorSimple.Direction.FORWARD;

    public static String odometryLeftName = "backLeft";
    public static String odometryCenterName = "backRight";
    public static String odometryRightName = "frontRight";

    public static DcMotorSimple.Direction odometryLeftDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction odometryCenterDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction odometryRightDirection = DcMotorSimple.Direction.REVERSE;

    public static MecanumController getMecanumController(HardwareMap hardwareMap) {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[] {
                frontLeftWheelName,
                frontRightWheelName,
                backLeftWheelName,
                backRightWheelName,
        });

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                frontLeftWheelDirection,
                frontRightWheelDirection,
                backLeftWheelDirection,
                backRightWheelDirection
        });

//        mecanumController.setPowerVectors(new Vector2D[]{
//                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993)
//        });

        mecanumController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        return mecanumController;
    }

    public static TeleOpController getTeleOpController(MecanumController mecanumController, Tracker tracker, HardwareMap hardwareMap) {
        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);

        teleOpController.configureIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        teleOpController.setSpeedController((gamepad) -> {
            if (gamepad.right_trigger > 0.1) {
                return 1.0;
            }
            return 0.6;
        });

        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY);

        return teleOpController;
    }

    public static Tracker getTracker(HardwareMap hardwareMap) {
        return new Tracker.TrackerBuilder(
                hardwareMap,
                ODOMETRY_TICKS_PER_INCH,
                FORWARD_OFFSET,
                ODOMETRY_WIDTH,
                odometryLeftName,
                odometryCenterName,
                odometryRightName,
                odometryLeftDirection,
                odometryCenterDirection,
                odometryRightDirection
        ).build();
    }
}