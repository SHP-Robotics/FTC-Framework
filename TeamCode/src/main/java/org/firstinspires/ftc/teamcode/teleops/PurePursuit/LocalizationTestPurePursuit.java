package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.SpeedController;

@TeleOp()
public class LocalizationTestPurePursuit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
            telemetry.update();
        }
    }
}
