package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.Vector2D;

@Config
public class PestoFTCConfig {
    public static double ODOMETRY_TICKS_PER_INCH = 505.316944316;
    public static double FORWARD_OFFSET = -1.80413492126;
    public static double ODOMETRY_WIDTH = 6.49606;

    // TODO: tune these
    public static double DECELERATION = -73.8807308633;
    public static double MAX_VELOCITY = 39;

    public static final DcMotorSimple.Direction leftEncoderDirection = FORWARD;
    public static final DcMotorSimple.Direction centerEncoderDirection = REVERSE;
    public static final DcMotorSimple.Direction rightEncoderDirection = FORWARD;

    public static String leftName = "frontLeft";
    public static String centerName = "backLeft";
    public static String rightName = "frontRight";

    public static MecanumController getMecanumController(HardwareMap hardwareMap) {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[] {
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight"
        });

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        });

        Vector2D frontLeftPower = new Vector2D(1, 1);

        // TODO: get power vectors
        mecanumController.setPowerVectors(new Vector2D[]{
                Vector2D.scale(Vector2D.multiply(frontLeftPower, new Vector2D(+1, 1)), 1/frontLeftPower.getMagnitude()),
                Vector2D.scale(Vector2D.multiply(frontLeftPower, new Vector2D(-1, 1)), 1/frontLeftPower.getMagnitude()),
                Vector2D.scale(Vector2D.multiply(frontLeftPower, new Vector2D(-1, 1)), 1/frontLeftPower.getMagnitude()),
                Vector2D.scale(Vector2D.multiply(frontLeftPower, new Vector2D(+1, 1)), 1/frontLeftPower.getMagnitude()),
        });

        mecanumController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return mecanumController;
    }

    public static TeleOpController getTeleOpController(MecanumController mecanumController, Tracker tracker, HardwareMap hardwareMap) {
        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);

        teleOpController.configureIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        teleOpController.useTrackerIMU(tracker);

        teleOpController.setSpeedController((gamepad) -> {
            if (gamepad.dpad_left) {
                return 0.6;
            }
            return 1.0;
        });

        // TODO: tune max velo
        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY);

        return teleOpController;
    }

    public static Tracker getTracker(HardwareMap hardwareMap) {
        return new ThreeWheelOdometryTracker.TrackerBuilder(
                hardwareMap,
                ODOMETRY_TICKS_PER_INCH,
                FORWARD_OFFSET,
                ODOMETRY_WIDTH,
                leftName,
                centerName,
                rightName,
                leftEncoderDirection,
                centerEncoderDirection,
                rightEncoderDirection
        ).build();
    }
}
