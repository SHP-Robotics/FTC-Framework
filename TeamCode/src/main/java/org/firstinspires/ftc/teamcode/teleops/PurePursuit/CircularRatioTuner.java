package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@TeleOp()
public class CircularRatioTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.4)
                .build();
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap);
        mecanumPurePursuitController.setSpeedController(speedController);

        waitForStart();
        telemetry.addLine("Run OdometryWidthTuner first");
        telemetry.addLine("Rotate 10 full times");
        telemetry.update();

        while (opModeIsActive()) {
            mecanumPurePursuitController.drive(gamepad1);
            mecanumPurePursuitController.rotationTestingUpdateOdometry();

            if (gamepad1.b) {
                // CIRCULAR_RATIO * distanceTravelledRotationally = -getX();
                // CIRCULAR_RATIO = -getX()/(getR() * ODOMETRY_WIDTH / 2)
                telemetry.addData("Ideal Circular Ratio", (2 * mecanumPurePursuitController.getCurrentPosition().getX()) / (mecanumPurePursuitController.getCurrentPosition().getHeadingRadians() * Constants.ODOMETRY_WIDTH));
                telemetry.update();
                break;
            }
        }

        while (opModeIsActive()) {}
    }
}
