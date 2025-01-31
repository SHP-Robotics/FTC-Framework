package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Vector2D;

@Config
public class PestoFTCConfig {
    public static double ODOMETRY_TICKS_PER_INCH = 505.316944316;
    public static double FORWARD_OFFSET = 0.5;
    public static double ODOMETRY_WIDTH = 11.25;

    // TODO: tune these
    // distance traveled / (2 * velocity)
    public static double DECELERATION = 1.3;
    public static double MAX_VELOCITY = 52;

    public static final DcMotorSimple.Direction leftEncoderDirection = REVERSE;
    public static final DcMotorSimple.Direction centerEncoderDirection = FORWARD;
    public static final DcMotorSimple.Direction rightEncoderDirection = FORWARD;

    public static String leftName = "frontLeft";
    public static String centerName = "frontRight";
    public static String rightName = "right";

    public static MecanumController getMecanumController(HardwareMap hardwareMap) {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[] {
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight"
        });

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                FORWARD,
                REVERSE,
                FORWARD,
                REVERSE
        });

        Vector2D frontLeftPower = new Vector2D(45, 55);

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

    public static TeleOpController getTeleOpController(MecanumController mecanumController, DeterministicTracker tracker, HardwareMap hardwareMap) {
        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);

        teleOpController.configureIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        teleOpController.useTrackerIMU(tracker);

        teleOpController.setSpeedController((gamepad) -> {
            if (gamepad.left_stick_button) {
                return 1.0;
            }
            return 0.6;
        });

        // TODO: tune max velo
        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY);

        return teleOpController;
    }

    public static DeterministicTracker getTracker(HardwareMap hardwareMap) {
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
