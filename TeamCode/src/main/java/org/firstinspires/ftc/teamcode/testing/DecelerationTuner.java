package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

//@Disabled
@TeleOp
public class DecelerationTuner extends LinearOpMode {
    private MecanumController mecanumController;
    private Tracker tracker;
    private TeleOpController teleOpController;

    private Vector2D before;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        waitForStart();

        telemetry.addLine("drive at max speed, then press b to get deceleration value");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.updateOdometry();
            teleOpController.updateSpeed(gamepad1);
            before = tracker.getCurrentPosition().copy();
        }

        mecanumController.drive(0, 0, 0);

        while (opModeIsActive() && !isStopRequested()) {
            tracker.updateOdometry();
            telemetry.addData("deceleration", Vector2D.dist(before, tracker.getCurrentPosition()));
            telemetry.update();
        }
    }
}
