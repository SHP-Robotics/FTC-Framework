package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;

@TeleOp()
public class OdometryWidthTuner extends LinearOpMode {
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
            mecanumPurePursuitController.rotationTestingUpdateOdometry();

            if (gamepad1.b) {

                // ideal / real
                // 20 radians * ODOMETRY_WIDTH / real radians
                telemetry.addData("Ideal Odometry Width", Math.abs((20 * Math.PI * ODOMETRY_WIDTH) / mecanumPurePursuitController.getCurrentPosition().getHeadingRadians()));
                telemetry.update();
                break;
            }
        }

        while (opModeIsActive()) {}
    }
}
