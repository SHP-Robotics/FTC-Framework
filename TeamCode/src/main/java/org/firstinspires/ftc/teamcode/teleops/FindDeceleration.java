package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@Disabled
@TeleOp
public class FindDeceleration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        Vector2D velocity = Vector2D.ZERO;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.updateOdometry();

            Vector2D currentPosition = tracker.getCurrentPosition();
            Pose2D currentVelocity = tracker.getRobotVelocity();
            telemetry.addData("X", currentPosition.getX());
            telemetry.addData("Y", currentPosition.getY());
            telemetry.addData("Rotation", tracker.getCurrentHeading());
            telemetry.addLine();
            telemetry.addData("X velocity", currentVelocity.getX());
            telemetry.addData("Y velocity", currentVelocity.getY());
            telemetry.addData("Rotational velocity", currentVelocity.getHeadingRadians());
            telemetry.update();

            if (gamepad1.b) {
                tracker.reset();
            }

            if (gamepad1.a) {
                teleOpController.driveRobotCentric(0, 0, 0);
                velocity = currentVelocity.asVector();
                tracker.reset();
                break;
            }
        }

        while (opModeIsActive() && !isStopRequested()) {
            tracker.updateOdometry();

            telemetry.addData("X velocity", velocity.getX());
            telemetry.addData("Y velocity", velocity.getY());
            telemetry.addLine();
            telemetry.addLine("Deceleration = -(velocity ^ 2) / (2 * distance)");
            telemetry.addData("Deceleration", -(velocity.getMagnitude() * velocity.getMagnitude()) / 2 + " / distance");
            telemetry.addLine();
            telemetry.addData("Distance", tracker.getCurrentPosition().getMagnitude());
            telemetry.update();
        }
    }
}
