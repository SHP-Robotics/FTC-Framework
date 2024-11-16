package org.firstinspires.ftc.teamcode.testing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@Disabled
@TeleOp(name = "Just Drive")
public class JustDrive extends LinearOpMode {
    private MecanumController mecanumController;
    private Tracker tracker;
    private TeleOpController teleOpController;

    private PathFollower follower;

    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        Servo test = hardwareMap.get(Servo.class, "test");
        test.scaleRange(0, 1);

        elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()) {
            loopOpMode();
        }
    }

    public void loopOpMode() {
        tracker.updateOdometry();

        if (gamepad1.dpad_right) {
            teleOpController.resetIMU();
            tracker.reset();
        }

        mecanumController.setZeroPowerBehavior(gamepad1.b ? BRAKE: FLOAT);

        Vector2D position = tracker.getCurrentPosition();
        double heading = tracker.getCurrentHeading();

        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("r", heading);

        teleOpController.updateSpeed(gamepad1);
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }
}
