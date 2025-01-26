package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.shprobotics.pestocore.devices.GamepadKey.A;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.Y;
import static org.firstinspires.ftc.teamcode.FourBarSubsystem.FourBarState.UP;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;

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

import java.util.List;

public abstract class BaseTeleOp extends LinearOpMode {
    protected MecanumController mecanumController;
    protected DeterministicTracker tracker;
    protected TeleOpController teleOpController;

    protected FourBarSubsystem fourBarSubsystem;
//    protected WristSubsystem wristSubsystem;
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

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        fourBarSubsystem = new FourBarSubsystem(hardwareMap);
//        wristSubsystem = new WristSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        specimenSubsystem = new SpecimenSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);

        modules = hardwareMap.getAll(LynxModule.class);

        gamepadInterface = new GamepadInterface(gamepad1);

        vector = null;
        heading = 0;

        elapsedTime = new ElapsedTime();

//        wristSubsystem.setState(WristSubsystem.WristState.UP);
        slideSubsystem.init();
        specimenSubsystem.init();

        fourBarSubsystem.setState(FourBarSubsystem.FourBarState.DOWN);
        fourBarSubsystem.update();

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

        if (gamepad1.dpad_down) {
            teleOpController.resetIMU();
            tracker.reset();
        }

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

//            if (slideSubsystem.getState() == HIGH)
//                wristSubsystem.setState(WristSubsystem.WristState.DEPOSIT);
//            else
//                wristSubsystem.setState(WristSubsystem.WristState.UP);

        if (gamepadInterface.isKeyDown(LEFT_BUMPER)) {
            if (fourBarSubsystem.getState() == UP)
                slideSubsystem.setState(slideSubsystem.getState().increment());
            fourBarSubsystem.setState(fourBarSubsystem.getState().increment());
        }

        if (gamepadInterface.isKeyDown(RIGHT_BUMPER)) {
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

        if (gamepadInterface.isKeyDown(Y))
            specimenSubsystem.increment();
        else if (gamepadInterface.isKeyDown(A))
            specimenSubsystem.decrement();

        if (gamepad1.dpad_right) {
            specimenSubsystem.setMode(RUN_WITHOUT_ENCODER);
            specimenSubsystem.setPower(-0.5);
            mecanumController.drive(0, 0, 0);

            while (gamepad1.dpad_right) {
            }

            specimenSubsystem.setPower(0);
            specimenSubsystem.setState(SpecimenSubsystem.SpecimenState.INTAKE);
            specimenSubsystem.init();
        }

        telemetry.addData("Radians", teleOpController.getHeading());

        fourBarSubsystem.update();
        intakeSubsystem.update();
        slideSubsystem.update();
//        wristSubsystem.update();

        specimenSubsystem.update();
        clawSubsystem.update();

        fourBarSubsystem.updateTelemetry(telemetry);
        intakeSubsystem.updateTelemetry(telemetry);
        slideSubsystem.updateTelemetry(telemetry);
//        wristSubsystem.updateTelemetry(telemetry);

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
