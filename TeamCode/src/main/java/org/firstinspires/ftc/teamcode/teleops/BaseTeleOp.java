package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_DOWN;
import static com.shprobotics.pestocore.devices.GamepadKey.DPAD_UP;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.HIGH;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;
import static org.firstinspires.ftc.teamcode.teleops.BaseTeleOp.SystemMode.SAMPLE;
import static org.firstinspires.ftc.teamcode.teleops.BaseTeleOp.SystemMode.SPECIMEN;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SlideSubsystem;
import org.firstinspires.ftc.teamcode.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.StaticVariables;
import org.firstinspires.ftc.teamcode.WristSubsystem;

import java.util.List;

public abstract class BaseTeleOp extends LinearOpMode {
    // intake, right trigger
    // outtake, left trigger

    // slide up, four bar up, right bumper
    // slide down, four bar down, left bumper

    // xmode, square

    protected MecanumController mecanumController;
    protected DeterministicTracker tracker;
    protected TeleOpController teleOpController;

    protected FourBarSubsystem fourBarSubsystem;
    protected WristSubsystem wristSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected SlideSubsystem slideSubsystem;

    protected SpecimenSubsystem specimenSubsystem;
    protected ClawSubsystem clawSubsystem;

    protected List<LynxModule> modules;

    protected GamepadInterface gamepadInterface;

    protected Vector2D vector;
    protected double heading;

    protected PathFollower follower;

    protected ElapsedTime elapsedTime;

    public enum SystemMode {
        SAMPLE,
        SPECIMEN;

        public SystemMode toggle() {
            if (this == SAMPLE)
                return SPECIMEN;
            return SAMPLE;
        }
    }

    protected SystemMode systemMode;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        wristSubsystem = new WristSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        specimenSubsystem = new SpecimenSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);

        modules = hardwareMap.getAll(LynxModule.class);

        gamepadInterface = new GamepadInterface(gamepad1);

        vector = null;
        heading = 0;

        elapsedTime = new ElapsedTime();

        systemMode = SPECIMEN;

        wristSubsystem.setState(WristSubsystem.WristState.UP);
//        slideSubsystem.init();
        specimenSubsystem.init();

        for (LynxModule module: modules)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        tracker.reset(StaticVariables.HEADING);
        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            loopOpMode();
        }

        StaticVariables.HEADING = tracker.getCurrentPosition().getHeadingRadians();
    }

    public void loopOpMode() {
        for (LynxModule module: modules)
            module.clearBulkCache();

        gamepadInterface.update();
        tracker.update();

//        if (gamepad1.x && vector != null) {
//            PathContainer path = new PathContainer.PathContainerBuilder()
//                    .addCurve(
//                            new BezierCurve(new Vector2D[]{
//                                    tracker.getCurrentPosition().asVector(),
//                                    vector
//                            }),
//                            new ParametricHeading(
//                                    new double[]{
//                                            tracker.getCurrentPosition().getHeadingRadians(),
//                                            heading
//                                    }
//                            )
//                    )
//                    .setIncrement(0.01)
//                    .build();
//
//            follower = new PathFollower.PathFollowerBuilder(
//                    mecanumController,
//                    tracker,
//                    path)
//                    .setDeceleration(PestoFTCConfig.DECELERATION)
//                    .setHeadingPID( new PID(2, 0, 0))
//                    .setEndpointPID(new PID(kP, 0, kD))
//                    .setSpeed(speed)
//                    .build();
//
//            slideSubsystem.setState(HIGH);
//            fourBarSubsystem.setState(UP);
//
//            slideSubsystem.update();
//            fourBarSubsystem.update();
//
//            while (opModeIsActive() && gamepad1.x) {
//                loopReturnToHome();
//            }
//        }

        if (gamepad1.dpad_right) {
            teleOpController.resetIMU();
            tracker.reset();
        }

        mecanumController.setZeroPowerBehavior(gamepad1.touchpad ? BRAKE: FLOAT);

        if (gamepadInterface.isKeyDown(DPAD_UP))
            systemMode = systemMode.toggle();

        if (systemMode == SAMPLE) {
            if (gamepadInterface.isKeyDown(DPAD_DOWN))
                wristSubsystem.setState(wristSubsystem.getState().cycle());

            if (gamepadInterface.isKeyDown(RIGHT_BUMPER)) {
                if (slideSubsystem.getState() == HIGH)
                    fourBarSubsystem.setState(fourBarSubsystem.getState().increment());
                slideSubsystem.setState(slideSubsystem.getState().increment());
            }

            if (gamepadInterface.isKeyDown(LEFT_BUMPER)) {
                if (slideSubsystem.getState() == INTAKE)
                    fourBarSubsystem.setState(fourBarSubsystem.getState().decrement());
                slideSubsystem.setState(slideSubsystem.getState().decrement());
            }

            if (gamepad1.left_trigger > 0.9) {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTTAKE);

                vector = tracker.getCurrentPosition().asVector();
                heading = tracker.getCurrentPosition().getHeadingRadians();
            } else if (gamepad1.right_trigger > 0.05)
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
            else if (slideSubsystem.getState() != INTAKE)
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.HOLD);
            else
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);

            if (gamepad1.dpad_left) {
                slideSubsystem.setMode(RUN_WITHOUT_ENCODER);
                slideSubsystem.setPower(-0.5);
                mecanumController.drive(0, 0, 0);

                while (gamepad1.dpad_left) {
                }

                slideSubsystem.setPower(0);
                slideSubsystem.setState(INTAKE);
                slideSubsystem.init();
            }
        } else {
            if (gamepadInterface.isKeyDown(RIGHT_BUMPER))
                specimenSubsystem.increment();
            else if (gamepadInterface.isKeyDown(LEFT_BUMPER))
                specimenSubsystem.decrement();

            if (gamepad1.left_trigger > 0.9) {
                clawSubsystem.setState(ClawSubsystem.ClawState.OPEN);

                vector = tracker.getCurrentPosition().asVector();
                heading = tracker.getCurrentPosition().getHeadingRadians();
            } else if (gamepad1.right_trigger > 0.05) {
                clawSubsystem.setState(ClawSubsystem.ClawState.CLOSE);
            }

            if (gamepad1.dpad_left) {
                specimenSubsystem.setMode(RUN_WITHOUT_ENCODER);
                specimenSubsystem.setPower(-0.5);
                mecanumController.drive(0, 0, 0);

                while (gamepad1.dpad_left) {
                }

                specimenSubsystem.setPower(0);
                specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.INTAKE);
                specimenSubsystem.init();
            }
        }

        telemetry.addData("Radians", teleOpController.getHeading());

        fourBarSubsystem.update();
//        intakeSubsystem.update();
//        slideSubsystem.update();
//        wristSubsystem.update();

        specimenSubsystem.update();
        clawSubsystem.update();

        fourBarSubsystem.updateTelemetry(telemetry);
        intakeSubsystem.updateTelemetry(telemetry);
        slideSubsystem.updateTelemetry(telemetry);
        wristSubsystem.updateTelemetry(telemetry);

        specimenSubsystem.updateTelemetry(telemetry);
        clawSubsystem.updateTelemetry(telemetry);

        teleOpController.updateSpeed(gamepad1);

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }

    public void loopReturnToHome() {
        tracker.update();
        follower.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }
}
