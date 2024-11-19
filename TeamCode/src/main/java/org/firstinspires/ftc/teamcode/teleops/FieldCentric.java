package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.DOWN;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.GLIDE;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.UP;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.HIGH;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.LOW;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SlideSubsystem;

@Config
@TeleOp(name = "Field Centric")
public class FieldCentric extends LinearOpMode {
    private MecanumController mecanumController;
    private Tracker tracker;
    private TeleOpController teleOpController;

    private FourBarSubsystem fourBarSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private SlideSubsystem slideSubsystem;

    private GamepadInterface gamepadInterface;

    private Vector2D vector;
    private double heading;

    private PathFollower follower;

    private ElapsedTime elapsedTime;

    public static double speed = 1.0;
    public static double kP = 0.1;
    public static double kD = 0.0;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        gamepadInterface = new GamepadInterface(gamepad1);

        vector = null;
        heading = 0;

        elapsedTime = new ElapsedTime();
        slideSubsystem.init();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()) {
            loopOpMode();
        }
    }

    public void loopOpMode() {
        gamepadInterface.update();
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
                    .setHeadingPID( new PID(2, 0, 0))
                    .setEndpointPID(new PID(kP, 0, kD))
                    .setSpeed(speed)
                    .build();

//            slideSubsystem.setState(HIGH);
//            fourBarSubsystem.setState(UP);

//            slideSubsystem.update();
//            fourBarSubsystem.update();

            while (opModeIsActive() && gamepad1.x) {
                loopReturnToHome();
            }
        }

        if (gamepad1.dpad_right) {
            teleOpController.resetIMU();
            tracker.reset();
        }

        mecanumController.setZeroPowerBehavior(gamepad1.b ? BRAKE: FLOAT);

        if (gamepadInterface.isKeyDown(RIGHT_BUMPER)) {
            if (fourBarSubsystem.getState() == GLIDE) fourBarSubsystem.setState(DOWN);
            if (slideSubsystem.getState() == LOW) slideSubsystem.setState(INTAKE);
            if (slideSubsystem.getState() == HIGH) slideSubsystem.setState(LOW);
            if (slideSubsystem.getState() == INTAKE && fourBarSubsystem.getState() == UP) fourBarSubsystem.setState(GLIDE);
        }

        if (gamepadInterface.isKeyDown(LEFT_BUMPER)) {
            if (fourBarSubsystem.getState() == UP) slideSubsystem.setState(LOW);
            if (fourBarSubsystem.getState() == GLIDE) fourBarSubsystem.setState(UP);
            if (fourBarSubsystem.getState() == DOWN) fourBarSubsystem.setState(GLIDE);
            if (slideSubsystem.getState() == LOW) slideSubsystem.setState(HIGH);
        }

        if (gamepad1.left_trigger > 0.9) {
            intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTAKE);

            vector = tracker.getCurrentPosition().copy();
            heading = tracker.getCurrentHeading();
        } else if (gamepad1.right_trigger > 0.05) {
            intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
        } else if (slideSubsystem.getState() != INTAKE) {
            intakeSubsystem.setState(IntakeSubsystem.IntakeState.HOLD);
        } else {
            intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
        }

        if (gamepad1.dpad_left) {
            slideSubsystem.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideSubsystem.setPower(-0.5);

            while (gamepad1.dpad_left) {}

            slideSubsystem.setPower(0);
            slideSubsystem.setState(INTAKE);
            slideSubsystem.init();
        }

        telemetry.addData("Radians", teleOpController.getHeading());

        fourBarSubsystem.update();
        intakeSubsystem.update();
        slideSubsystem.update();

        fourBarSubsystem.updateTelemetry(telemetry);
        intakeSubsystem. updateTelemetry(telemetry);
        slideSubsystem.  updateTelemetry(telemetry);

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
