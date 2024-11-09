package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.DOWN;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.UP;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.HIGH;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "Robot Centric")
public class RobotCentric extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        FourBarSubsystem fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        SlideSubsystem slideSubsystem = new SlideSubsystem(hardwareMap);

        GamepadInterface gamepadInterface = new GamepadInterface(gamepad1);

        Vector2D vector = null;
        double heading = 0;

        waitForStart();

        slideSubsystem.init();

        while (opModeIsActive()) {
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

                PathFollower follower = new PathFollower.PathFollowerBuilder(
                        mecanumController,
                        tracker,
                        path)
                        .setDeceleration(PestoFTCConfig.DECELERATION)
                        .setHeadingPID( new PID(0.1, 0, 0))
                        .setEndpointPID(new PID(0.1, 0, 0))
                        .setSpeed(0.3)
                        .build();

                while (opModeIsActive() && gamepad1.x) {
                    tracker.updateOdometry();
                    follower.update();
                }
            }

            if (gamepad1.dpad_right) {
                teleOpController.resetIMU();
                tracker.reset();
            }

            mecanumController.setZeroPowerBehavior(gamepad1.b ? BRAKE: FLOAT);

            if (gamepadInterface.isKeyDown(LEFT_BUMPER)) {
                if (slideSubsystem.getState() == INTAKE) fourBarSubsystem.setState(DOWN);
                slideSubsystem.setState(INTAKE);
            }

            if (gamepadInterface.isKeyDown(RIGHT_BUMPER)) {
                if (fourBarSubsystem.getState() == UP) slideSubsystem.setState(HIGH);
                fourBarSubsystem.setState(UP);
            }

            if ((gamepad1.left_trigger > 0.9) == (gamepad1.right_trigger > 0.1)) {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
            } else if (gamepad1.left_trigger > 0.9) {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTAKE);

                vector = tracker.getCurrentPosition();
                heading = tracker.getCurrentHeading();
            } else {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
            }

            telemetry.addData("Radians", teleOpController.getHeading());

            fourBarSubsystem.updateTelemetry(telemetry);
            intakeSubsystem. updateTelemetry(telemetry);
            slideSubsystem.  updateTelemetry(telemetry);

            teleOpController.updateSpeed(gamepad1);
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.update();
        }
    }
}
