package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.MecanumTracker;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

@Config
public class PestoFTCConfig {
    public static double ODOMETRY_TICKS_PER_INCH = 336.877962878;
    public static double FORWARD_OFFSET = -0.0294066;
    public static double ODOMETRY_WIDTH = 23.36825;
    public static double DECELERATION = -73.8807308633;
    public static double MAX_VELOCITY = 39;

//    public static final DcMotorSimple.Direction leftEncoderDirection = REVERSE;
//    public static final DcMotorSimple.Direction centerEncoderDirection = FORWARD;
//    public static final DcMotorSimple.Direction rightEncoderDirection = FORWARD;
//
//    public static String leftName = "left";
//    public static String centerName = "center";
//    public static String rightName = "right";

    public static final DcMotorSimple.Direction frontLeftDirection = REVERSE;
    public static final DcMotorSimple.Direction frontRightDirection = FORWARD;
    public static final DcMotorSimple.Direction backLeftDirection = REVERSE;
    public static final DcMotorSimple.Direction backRightDirection = FORWARD;

    public static String frontLeftName = "frontLeft";
    public static String frontRightName = "frontRight";
    public static String backLeftName = "backLeft";
    public static String backRightName = "backRight";

    public static MecanumController getMecanumController(HardwareMap hardwareMap) {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[] {
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight"
        });

        //mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
          //      DcMotorSimple.Direction.FORWARD,
            //    DcMotorSimple.Direction.FORWARD,
              //  DcMotorSimple.Direction.REVERSE ,
                //DcMotorSimple.Direction.REVERSE
       // });

//        mecanumController.setPowerVectors(new Vector2D[]{
//                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993)
//        });

        mecanumController.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return mecanumController;
    }

    public static TeleOpController getTeleOpController(MecanumController mecanumController, MecanumTracker tracker, HardwareMap hardwareMap) {
        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);

        teleOpController.configureIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        teleOpController.setSpeedController((gamepad) -> {
            if (gamepad.right_trigger > 0.1) {
                return 1.0;
            }
            return 0.6;
        });

//        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY);
        teleOpController.deactivateCentripetalForce();

        return teleOpController;
    }

   public static MecanumTracker getTracker(HardwareMap hardwareMap) {
        return new MecanumTracker.TrackerBuilder(hardwareMap,
                ODOMETRY_TICKS_PER_INCH,
                new Vector2D(7, 6.25),
                frontLeftName,
                frontRightName,
                backLeftName,
                backRightName,
                frontLeftDirection,
                frontRightDirection,
                backLeftDirection,
                backRightDirection).build();

    }
}