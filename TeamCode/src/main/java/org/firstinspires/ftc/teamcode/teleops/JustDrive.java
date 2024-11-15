package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@TeleOp(name = "Just Drive")
public class JustDrive extends LinearOpMode {
    private MecanumController mecanumController;
    private Tracker tracker;
    private TeleOpController teleOpController;

//    private GamepadInterface gamepadInterface;

    private Vector2D vector;
    private double heading;

    private PathFollower follower;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

//        gamepadInterface = new GamepadInterface(gamepad1);

        vector = null;
        heading = 0;

        Servo test = hardwareMap.get(Servo.class, "test");
        test.scaleRange(0, 1);

        elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()) {
            loopOpMode();
        }
    }

    public void loopOpMode() {
//        gamepadInterface.update();
        tracker.updateOdometry();

        if (gamepad1.x && vector != null) {
            PathContainer path = new PathContainer.PathContainerBuilder()
                    .addCurve(
                            new BezierCurve(new Vector2D[]{
                                    tracker.getCurrentPosition(),
                                    vector
                            }),
                            new ParametricHeading(
                                    tracker.getCurrentHeading(),
                                    heading
                            )
                    )
                    .setIncrement(0.01)
                    .build();

            follower = new PathFollower.PathFollowerBuilder(
                    mecanumController,
                    tracker,
                    path)
                    .setDeceleration(PestoFTCConfig.DECELERATION)
                    .setHeadingPID( new PID(0.1, 0, 0))
                    .setEndpointPID(new PID(0.1, 0, 0))
                    .setSpeed(0.3)
                    .build();

            while (opModeIsActive() && gamepad1.x) {
                loopReturnToHome();
            }
        }

        if (gamepad1.dpad_right) {
            teleOpController.resetIMU();
            tracker.reset();
        }

        mecanumController.setZeroPowerBehavior(gamepad1.b ? BRAKE: FLOAT);

        telemetry.addData("Radians", teleOpController.getHeading());

        teleOpController.updateSpeed(gamepad1);
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }

    public void loopReturnToHome() {
        tracker.updateOdometry();
        follower.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }
}
