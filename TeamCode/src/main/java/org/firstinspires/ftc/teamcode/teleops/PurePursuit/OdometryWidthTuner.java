package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@TeleOp()
public class OdometryWidthTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.4)
                .build();
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap);
        mecanumPurePursuitController.setSpeedController(speedController);
        mecanumPurePursuitController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.addLine("1. Rotate 10 full times.");
        telemetry.addLine("2. Press B when complete.");
        telemetry.update();

        while (opModeIsActive()) {
            mecanumPurePursuitController.drive(gamepad1);
            mecanumPurePursuitController.rotationTestingUpdateOdometry();

            if (gamepad1.b) {
                // predicted too far -> increase width
                // predicted too short -> decrease width
                telemetry.addData("Ideal Odometry Width", Constants.ODOMETRY_WIDTH * Math.abs(mecanumPurePursuitController.getCurrentPosition().getHeadingRadians() / (10 * Math.PI)));
                telemetry.update();
                break;
            }
        }

        while (opModeIsActive()) {}
    }
}
