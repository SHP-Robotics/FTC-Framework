package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;

@TeleOp()
public class LocalizationTestPurePursuit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.4)
                .build();
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap);
        mecanumPurePursuitController.setSpeedController(speedController);
        mecanumPurePursuitController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
