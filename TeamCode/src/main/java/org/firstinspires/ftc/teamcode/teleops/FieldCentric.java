package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.shprobotics.pestocore.devices.GamepadKey.LEFT_BUMPER;
import static com.shprobotics.pestocore.devices.GamepadKey.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.ClawSubsystem.ClawState.CLOSE;
import static org.firstinspires.ftc.teamcode.ClawSubsystem.ClawState.OPEN;
import static org.firstinspires.ftc.teamcode.SlideSubsystem.SlideState.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;

import org.firstinspires.ftc.teamcode.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.SlideSubsystem;

@TeleOp(name = "Field Centric")
public class FieldCentric extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    private TeleOpController teleOpController;

    private ClawSubsystem clawSubsystem;
    private SlideSubsystem slideSubsystem;

    private GamepadInterface gamepadInterface;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        clawSubsystem = new ClawSubsystem(hardwareMap);
        slideSubsystem = new SlideSubsystem(hardwareMap);

        gamepadInterface = new GamepadInterface(gamepad1);

        elapsedTime = new ElapsedTime();
//        slideSubsystem.init();

        waitForStart();
        slideSubsystem.init();

        elapsedTime.reset();

        while (opModeIsActive()) {
            loopOpMode();
        }
    }

    public void loopOpMode() {
        gamepadInterface.update();
        tracker.update();

        if (gamepad1.dpad_right) {
            teleOpController.resetIMU();
            tracker.reset();
        }

        mecanumController.setZeroPowerBehavior(gamepad1.b ? BRAKE: FLOAT);

        if (gamepadInterface.isKeyDown(RIGHT_BUMPER)) slideSubsystem.increment();
        else if (gamepadInterface.isKeyDown(LEFT_BUMPER)) slideSubsystem.decrement();

        if (gamepad1.left_trigger > 0.9)
            clawSubsystem.setState(OPEN);
        else if (gamepad1.right_trigger > 0.1)
            clawSubsystem.setState(CLOSE);

        if (gamepad1.dpad_left) {
            slideSubsystem.setPower(-0.5);

            while (gamepad1.dpad_left) {}

            slideSubsystem.setPower(0);
            slideSubsystem.setState(INTAKE);
        }

        telemetry.addData("Radians", teleOpController.getHeading());

        clawSubsystem.update();
        slideSubsystem.update();

        clawSubsystem. updateTelemetry(telemetry);
        slideSubsystem.  updateTelemetry(telemetry);

        teleOpController.updateSpeed(gamepad1);
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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
