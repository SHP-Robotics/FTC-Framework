package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        DeterministicTracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        tracker.reset();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            tracker.update();
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.b) {
                tracker.reset();
                teleOpController.resetIMU();
            }

            telemetry.addData("X", tracker.getCurrentPosition().getX());
            telemetry.addData("Y", tracker.getCurrentPosition().getY());
            telemetry.addData("R", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
