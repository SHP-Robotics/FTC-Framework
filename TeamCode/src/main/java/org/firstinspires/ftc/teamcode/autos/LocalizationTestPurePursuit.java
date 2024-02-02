package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;

@TeleOp()
public class LocalizationTestPurePursuit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double MECANUM_WIDTH = 14;
        double ODOMETRY_WIDTH = 11;

        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(
                hardwareMap,
                MECANUM_WIDTH,
                ODOMETRY_WIDTH);

        waitForStart();

        while (opModeIsActive()) {
            mecanumPurePursuitController.drive(gamepad1);
            mecanumPurePursuitController.updateOdometry();

            telemetry.addData("x", mecanumPurePursuitController.getCurrentPosition().getX());
            telemetry.addData("y", mecanumPurePursuitController.getCurrentPosition().getY());
            telemetry.addData("r", mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
