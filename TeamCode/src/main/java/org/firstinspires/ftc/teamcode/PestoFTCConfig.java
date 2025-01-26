package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

@Config
public class PestoFTCConfig {
    public static double ODOMETRY_TICKS_PER_INCH = 505.316944316;
    public static double FORWARD_OFFSET = 0.8;
    public static double ODOMETRY_WIDTH = 6.65;

    // TODO: tune these
    // distance traveled / velocity
    public static double DECELERATION = 0.9;
    public static double MAX_VELOCITY = 52;

    public static double endpointKP = 0.005;
    public static double headingKP = 0.8;

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

    public static TeleOpController getTeleOpController(MecanumController mecanumController, DeterministicTracker tracker, HardwareMap hardwareMap) {
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

    public static PathFollower.PathFollowerBuilder generatePathFollower(PathContainer pathContainer, MecanumController mecanumController, DeterministicTracker tracker) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(endpointKP, 0, 0))
                .setHeadingPID(new PID(headingKP, 0, 0))
                .setDeceleration(DECELERATION)
                .setSpeed(1.0);

//                .setEndTolerance(0.4, Math.toRadians(2))
//                .setEndVelocityTolerance(4);
    }

    public static Runnable getPlaceSpecimen(DeterministicTracker tracker, SpecimenSubsystem specimenSubsystem, ClawSubsystem clawSubsystem, ElapsedTime elapsedTime, LinearOpMode linearOpMode) {
        return () -> {
            elapsedTime.reset();
            while (linearOpMode.opModeIsActive() && elapsedTime.seconds() < 0.5) {
                specimenSubsystem.update();
                tracker.update();
            }
            specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.DEPOSIT_HIGH);
            elapsedTime.reset();
            while (linearOpMode.opModeIsActive() && elapsedTime.seconds() < 1.0) {
                specimenSubsystem.update();
                tracker.update();
            }

            elapsedTime.reset();
            clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);

            while (linearOpMode.opModeIsActive() && elapsedTime.seconds() < 0.5) {
                clawSubsystem.update();
                tracker.update();
            }

            specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.INTAKE);
        };
    }

    public static Runnable getGrabSpecimen(DeterministicTracker tracker, SpecimenSubsystem specimenSubsystem, ClawSubsystem clawSubsystem, ElapsedTime elapsedTime, LinearOpMode linearOpMode) {
        return () -> {
            clawSubsystem.setState(ClawSubsystem.ClawState.CLOSE);
            clawSubsystem.update();

            elapsedTime.reset();

            while (linearOpMode.opModeIsActive() && elapsedTime.seconds() < 0.2) {
                tracker.update();
            }

            specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.HIGH);

            elapsedTime.reset();
            while (linearOpMode.opModeIsActive() && elapsedTime.seconds() < 0.5) {
                tracker.update();
                specimenSubsystem.update();
            }
        };
    }
}
