package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@TeleOp()
public class OdometryWidthTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);

        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.7)
                .build();
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setSpeedController(speedController);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.addLine("1. Rotate 10 full times counter-clockwise.");
        telemetry.addLine("2. Press B when complete.");
        telemetry.update();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
            purePursuitFollower.rotationTestingUpdateOdometry();

            if (gamepad1.b) {
                // predicted too far -> increase width
                // predicted too short -> decrease width
                double newOdometryWidth = Constants.ODOMETRY_WIDTH * Math.abs((purePursuitFollower.getCurrentPosition().getHeadingRadians()-(Math.PI/2)) / (20 * Math.PI));
                telemetry.addData("3. Set odometry width to", newOdometryWidth);
                telemetry.update();
                Constants.ODOMETRY_WIDTH = newOdometryWidth;
                break;
            }
        }

        while (opModeIsActive()) {}
    }
}
