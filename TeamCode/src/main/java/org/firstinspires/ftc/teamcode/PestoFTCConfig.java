package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;

@Config
public class PestoFTCConfig {
    public static double ODOMETRY_TICKS_PER_INCH = 505.3169;
    public static double FORWARD_OFFSET = 0;
    public static double ODOMETRY_WIDTH = 14.35782;
    public static double DECELERATION = 0.455;
    public static double MAX_VELOCITY = 46;

    public static final DcMotorSimple.Direction leftEncoderDirection = FORWARD;
    public static final DcMotorSimple.Direction centerEncoderDirection = REVERSE;
    public static final DcMotorSimple.Direction rightEncoderDirection = FORWARD;

    public static String leftName = "leftFront";
    public static String centerName = "rightFront";
    public static String rightName = "rightRear";

    public static MecanumController getMecanumController(HardwareMap hardwareMap) {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[] {
                "leftFront",
                "rightFront",
                "leftRear",
                "rightRear"
        });

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                REVERSE,
                FORWARD,
                REVERSE,
                FORWARD
        });

//            mecanumController.setPowerVectors(new Vector2D[]{
//                    Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993),
//                    Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                    Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                    Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993)
//            });
//
//            mecanumController.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return mecanumController;
    }

    public static TeleOpController getTeleOpController(MecanumController mecanumController, DeterministicTracker tracker, HardwareMap hardwareMap) {
        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);
        teleOpController.useTrackerIMU(tracker);

        teleOpController.configureIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        teleOpController.setSpeedController((gamepad) -> {
            if (gamepad.right_bumper) {
                return 0.6;
            }
            return 1.0;
        });

        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY); //TODO PUT IN NOTEBOOK

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