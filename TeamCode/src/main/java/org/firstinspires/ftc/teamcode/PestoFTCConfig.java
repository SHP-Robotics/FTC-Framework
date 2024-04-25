package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.Vector2D;

public class PestoFTCConfig {
    public static final double ODOMETRY_TICKS_PER_INCH = 336.877962878;
    public static final double FORWARD_OFFSET = -0.0294066;
    public static final double ODOMETRY_WIDTH = 23.36825;

    public static final DcMotorSimple.Direction leftEncoderDirection = REVERSE;
    public static final DcMotorSimple.Direction centerEncoderDirection = FORWARD;
    public static final DcMotorSimple.Direction rightEncoderDirection = FORWARD;

    public static final String leftName = "left";
    public static final String centerName = "center";
    public static final String rightName = "right";

    public static MecanumController getMecanumController(HardwareMap hardwareMap) {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[] {
                "leftFront",
                "rightFront",
                "leftRear",
                "rightRear"
        });

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        });

        mecanumController.setPowerVectors(new Vector2D[]{
                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993),
                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993)
        });

        mecanumController.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return mecanumController;
    }

    public static TeleOpController getTeleOpController(MecanumController mecanumController, HardwareMap hardwareMap) {
        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);

        teleOpController.configureIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        teleOpController.setSpeedController(gamepad -> {
            if (gamepad.right_trigger > 0.1) {
                return 1.0;
            }
            return 0.6;
        });

        return teleOpController;
    }

    public static Tracker getTracker(HardwareMap hardwareMap) {
        return new Tracker.TrackerBuilder(
                hardwareMap,
                ODOMETRY_TICKS_PER_INCH,
                FORWARD_OFFSET,
                ODOMETRY_WIDTH,
                leftEncoderDirection,
                centerEncoderDirection,
                rightEncoderDirection,
                leftName,
                centerName,
                rightName
        ).build();
    }
}
