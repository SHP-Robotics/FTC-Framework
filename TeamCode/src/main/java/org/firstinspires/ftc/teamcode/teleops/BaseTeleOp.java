package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.shprobotics.pestocore.devices.GamepadKey.A;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.Y;
import static org.firstinspires.ftc.teamcode.SampleSlideSubsystem.SlideState.INTAKE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.FourBarSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.SampleSlideSubsystem;
import org.firstinspires.ftc.teamcode.SpecimenClawSubsystem;
import org.firstinspires.ftc.teamcode.SpecimenSlideSubsystem;
import org.firstinspires.ftc.teamcode.StaticVariables;

import java.util.List;

public abstract class BaseTeleOp extends LinearOpMode {
    protected MecanumController mecanumController;
    protected DeterministicTracker tracker;
    protected TeleOpController teleOpController;

    protected FourBarSubsystem fourBarSubsystem;
    protected SampleIntakeSubsystem sampleIntakeSubsystem;
    protected SampleSlideSubsystem sampleSlideSubsystem;

    protected SpecimenSlideSubsystem specimenSlideSubsystem;
    protected SpecimenClawSubsystem specimenClawSubsystem;

    protected List<LynxModule> modules;

    protected GamepadInterface gamepadInterface;

    protected Vector2D vector;
    protected double heading;

    protected PathFollower follower;

    protected ElapsedTime elapsedTime;
    protected ElapsedTime specimenUpTimer = null;
    protected ElapsedTime specimenDownTimer = null;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        sampleIntakeSubsystem = new SampleIntakeSubsystem(hardwareMap);
        sampleSlideSubsystem = new SampleSlideSubsystem(hardwareMap);

        specimenSlideSubsystem = new SpecimenSlideSubsystem(hardwareMap);
        specimenClawSubsystem = new SpecimenClawSubsystem(hardwareMap);

        modules = hardwareMap.getAll(LynxModule.class);

        gamepadInterface = new GamepadInterface(gamepad1);

        vector = null;
        heading = 0;

        elapsedTime = new ElapsedTime();

        sampleSlideSubsystem.init();
        specimenSlideSubsystem.init();

        fourBarSubsystem.setState(FourBarSubsystem.FourBarState.DOWN);
        fourBarSubsystem.update();

        for (LynxModule module: modules)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

//        tracker.reset(StaticVariables.HEADING);
        tracker.reset();
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

        if (gamepad1.x) {
            teleOpController.resetIMU();
            tracker.reset();
        }

        if (gamepad1.dpad_up) {
            sampleSlideSubsystem.increment();
            fourBarSubsystem.setState(FourBarSubsystem.FourBarState.UP);
        } else if (gamepad1.dpad_down)
            sampleSlideSubsystem.decrement();

        mecanumController.setZeroPowerBehavior(gamepad1.touchpad ? BRAKE: FLOAT);

//            if (gamepad1.x && vector != null) {
//                PathContainer path = new PathContainer.PathContainerBuilder()
//                        .addCurve(
//                                new BezierCurve(new Vector2D[]{
//                                        tracker.getCurrentPosition().asVector(),
//                                        vector
//                                }),
//                                new ParametricHeading(
//                                        new double[]{
//                                                tracker.getCurrentPosition().getHeadingRadians(),
//                                                heading
//                                        }
//                                )
//                        )
//                        .setIncrement(0.01)
//                        .build();
//
//                follower = new PathFollower.PathFollowerBuilder(
//                        mecanumController,
//                        tracker,
//                        path)
//                        .setDeceleration(PestoFTCConfig.DECELERATION)
//                        .setHeadingPID(new PID(PestoFTCConfig.headingKP, 0, 0))
//                        .setEndpointPID(new PID(PestoFTCConfig.endpointKP, 0, 0))
//                        .setSpeed(1.0)
//                        .build();
//
//                slideSubsystem.setState(HIGH);
//                fourBarSubsystem.setState(UP);
////                wristSubsystem.setState(WristSubsystem.WristState.DEPOSIT);
//
//                slideSubsystem.update();
//                fourBarSubsystem.update();
//
//                while (opModeIsActive() && gamepad1.x) {
//                    loopReturnToHome();
//                }
//            }

        if (gamepadInterface.isKeyDown(LEFT_BUMPER))
            fourBarSubsystem.increment();

        if (gamepadInterface.isKeyDown(RIGHT_BUMPER))
            fourBarSubsystem.decrement();

        if (gamepad1.left_trigger > 0.9) {
            sampleIntakeSubsystem.setState(SampleIntakeSubsystem.IntakeState.OUTTAKE);

            vector = tracker.getCurrentPosition().asVector();
            heading = tracker.getCurrentPosition().getHeadingRadians();
        } else if (gamepad1.right_trigger > 0.05)
            sampleIntakeSubsystem.setState(SampleIntakeSubsystem.IntakeState.INTAKE);
        else if (sampleSlideSubsystem.getState() != INTAKE)
            sampleIntakeSubsystem.setState(SampleIntakeSubsystem.IntakeState.HOLD);
        else
            sampleIntakeSubsystem.setState(SampleIntakeSubsystem.IntakeState.NEUTRAL);

        if (gamepad1.dpad_left) {
            sampleSlideSubsystem.setMode(RUN_WITHOUT_ENCODER);
            sampleSlideSubsystem.setPower(-0.5);
            mecanumController.drive(0, 0, 0);

            while (gamepad1.dpad_left) {
            }

            sampleSlideSubsystem.setPower(0);
            sampleSlideSubsystem.setState(INTAKE);
            sampleSlideSubsystem.init();
        }

        if (gamepadInterface.isKeyDown(Y)) {
            if (specimenClawSubsystem.getState() != SpecimenClawSubsystem.ClawState.CLOSE) {
                specimenUpTimer = new ElapsedTime();
                specimenUpTimer.reset();
                specimenDownTimer = null;
                specimenClawSubsystem.setState(SpecimenClawSubsystem.ClawState.CLOSE);
            } else {
                specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.HIGH);
                specimenUpTimer = null;
            }
        } else if (gamepadInterface.isKeyDown(A)) {
            if (specimenSlideSubsystem.getState() == SpecimenSlideSubsystem.SpecimenState.HIGH) {
                specimenDownTimer = new ElapsedTime();
                specimenDownTimer.reset();
                specimenUpTimer = null;

                specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.BELOW_HIGH);
            } else {
                if (specimenSlideSubsystem.getState() == SpecimenSlideSubsystem.SpecimenState.HIGH)
                    specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.BELOW_HIGH);
                else
                    specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.INTAKE);
                specimenClawSubsystem.setState(SpecimenClawSubsystem.ClawState.OPEN);
                specimenDownTimer = null;
            }
        }

        if (specimenUpTimer != null && specimenUpTimer.seconds() > 0.2) {
            specimenUpTimer = null;
            specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.HIGH);
        }

        if (specimenDownTimer != null && specimenDownTimer.seconds() > 0.4) {
            specimenDownTimer = null;
            specimenClawSubsystem.setState(SpecimenClawSubsystem.ClawState.OPEN);
            specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.INTAKE);
        }

        if (gamepad1.dpad_right) {
            specimenSlideSubsystem.setMode(RUN_WITHOUT_ENCODER);
            specimenSlideSubsystem.setPower(-0.5);
            mecanumController.drive(0, 0, 0);

            while (gamepad1.dpad_right) {
            }

            specimenSlideSubsystem.setPower(0);
            specimenSlideSubsystem.setState(SpecimenSlideSubsystem.SpecimenState.INTAKE);
            specimenSlideSubsystem.init();
        }

        telemetry.addData("Radians", teleOpController.getHeading());

        fourBarSubsystem.update();
        sampleIntakeSubsystem.update();
        sampleSlideSubsystem.update();

        specimenSlideSubsystem.update();
        specimenClawSubsystem.update();

        fourBarSubsystem.updateTelemetry(telemetry);
        sampleIntakeSubsystem.updateTelemetry(telemetry);
        sampleSlideSubsystem.updateTelemetry(telemetry);

        specimenSlideSubsystem.updateTelemetry(telemetry);
        specimenClawSubsystem.updateTelemetry(telemetry);

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
