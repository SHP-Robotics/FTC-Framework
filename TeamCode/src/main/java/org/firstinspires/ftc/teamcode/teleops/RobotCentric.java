package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.shprobotics.pestocore.devices.GamepadKey.B;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.X;
import static com.shprobotics.pestocore.devices.GamepadKey.Y;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.BELOW_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SampleSlideSubsystem;
import org.firstinspires.ftc.teamcode.SlideSubsystem;
import org.firstinspires.ftc.teamcode.SpecimenClawSubsystem;
import org.firstinspires.ftc.teamcode.WristSubsystem;

import java.util.List;

@TeleOp(name = "Robot Centric")
public class RobotCentric extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    private TeleOpController teleOpController;
    private List<LynxModule> modules;

    private ClawSubsystem clawSubsystem;
    private SlideSubsystem slideSubsystem;

    private SampleSlideSubsystem sampleSlideSubsystem;
    private WristSubsystem wristSubsystem;
    private SpecimenClawSubsystem specimenClawSubsystem;

    private GamepadInterface gamepadInterface;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        clawSubsystem = new ClawSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        sampleSlideSubsystem = new SampleSlideSubsystem(hardwareMap);
        wristSubsystem = new WristSubsystem(hardwareMap);
        specimenClawSubsystem = new SpecimenClawSubsystem(hardwareMap);

        gamepadInterface = new GamepadInterface(gamepad1);

        elapsedTime = new ElapsedTime();
        slideSubsystem.init();

        modules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module: modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        wristSubsystem.update();
        specimenClawSubsystem.update();

        waitForStart();
        slideSubsystem.init();

        elapsedTime.reset();

        while (opModeIsActive()) {
            loopOpMode();
        }
    }

    public void loopOpMode() {
        for (LynxModule module: modules) {
            module.clearBulkCache();
        }

        if (gamepad1.dpad_up) {
            teleOpController.useTrackerIMU(tracker);
            teleOpController.resetIMU();
        } else if (gamepad1.dpad_down) {
            teleOpController.useIMU();
            teleOpController.resetIMU();
        }

        gamepadInterface.update();
        tracker.update();

        if (gamepadInterface.isKeyDown(B))
            slideSubsystem.increment();
        else if (gamepadInterface.isKeyDown(X)) {
            if (slideSubsystem.getState() == BELOW_HIGH_RUNG) {
                specimenClawSubsystem.setState(SpecimenClawSubsystem.ClawState.OPEN);
            }
            slideSubsystem.decrement();
        }

        if (gamepadInterface.isKeyDown(Y))
            specimenClawSubsystem.toggle();

        if (gamepadInterface.isKeyDown(RIGHT_BUMPER))
            clawSubsystem.toggle();
        else if (gamepadInterface.isKeyDown(LEFT_BUMPER))
            wristSubsystem.toggle();

        sampleSlideSubsystem.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if (gamepad1.a) {
            teleOpController.resetIMU();
            tracker.reset();
        }

        mecanumController.setZeroPowerBehavior(gamepad1.b ? BRAKE: FLOAT);

        if (gamepad1.dpad_left) {
            slideSubsystem.setPower(-0.5);

            while (gamepad1.dpad_left) {}

            slideSubsystem.setPower(0);
            slideSubsystem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideSubsystem.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideSubsystem.setState(INTAKE);
        }

        if (gamepad1.dpad_right) {
            sampleSlideSubsystem.motor.setMode(STOP_AND_RESET_ENCODER);
            sampleSlideSubsystem.motor.setMode(RUN_USING_ENCODER);
        }

        telemetry.addData("Radians", teleOpController.getHeading());

        clawSubsystem.update();
        slideSubsystem.update();

        specimenClawSubsystem.update();
        wristSubsystem.update();

        clawSubsystem.updateTelemetry(telemetry);
        slideSubsystem.updateTelemetry(telemetry);

        sampleSlideSubsystem.updateTelemetry(telemetry);
        specimenClawSubsystem.updateTelemetry(telemetry);
        wristSubsystem.updateTelemetry(telemetry);

        teleOpController.updateSpeed(gamepad1);
        teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }

    public void loopReturnToHome() {
        tracker.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }
}
