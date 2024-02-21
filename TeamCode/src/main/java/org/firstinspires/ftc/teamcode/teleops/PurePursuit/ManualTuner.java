package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@TeleOp()
public class ManualTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean holdingDpadUp = false;
        boolean holdingDpadDown = false;
        boolean holdingDpadLeft = false;
        boolean holdingDpadRight = false;

        double additive = 1;

        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);

        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.4)
                .build();
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setSpeedController(speedController);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
            purePursuitFollower.updateOdometry();

            telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
            telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
            telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("Circular ratio", Constants.CIRCULAR_RATIO);
            telemetry.addData("Additive", additive);
            telemetry.update();

            if (gamepad1.b) {
                purePursuitFollower.reset();
            }

            if (gamepad1.dpad_up && !holdingDpadUp) {
                Constants.CIRCULAR_RATIO += additive;
            }

            if (gamepad1.dpad_down && !holdingDpadDown) {
                Constants.CIRCULAR_RATIO -= additive;
            }

            if (gamepad1.dpad_left && !holdingDpadLeft) {
                additive *= 10;
            }

            if (gamepad1.dpad_right && !holdingDpadRight) {
                additive /= 10;
            }

            holdingDpadUp = gamepad1.dpad_up;
            holdingDpadDown = gamepad1.dpad_down;
            holdingDpadLeft = gamepad1.dpad_left;
            holdingDpadRight = gamepad1.dpad_right;
        }
    }
}
