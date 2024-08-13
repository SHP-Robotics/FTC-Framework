package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.ODOMETRY_TICKS_PER_INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.Odometry;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@TeleOp
public class TuneForwardOffset extends LinearOpMode {
    public void runOpMode() {
        Odometry leftOdometry = new Odometry(
                (DcMotor)hardwareMap.get(PestoFTCConfig.odometryLeftName),
                ODOMETRY_TICKS_PER_INCH);

        leftOdometry.setDirection(PestoFTCConfig.odometryLeftDirection);

        Odometry rightOdometry = new Odometry(
                (DcMotor)hardwareMap.get(PestoFTCConfig.odometryRightName),
                ODOMETRY_TICKS_PER_INCH);

        rightOdometry.setDirection(PestoFTCConfig.odometryRightDirection);

        Odometry centerOdometry = new Odometry(
                (DcMotor)hardwareMap.get(PestoFTCConfig.odometryCenterName),
                ODOMETRY_TICKS_PER_INCH);

        centerOdometry.setDirection(PestoFTCConfig.odometryCenterDirection);

        leftOdometry.reset();
        rightOdometry.reset();
        centerOdometry.reset();

        double totalDistanceRotated = 0;
        double totalMeasuredX = 0;

        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.y) {
            teleOpController.updateSpeed(gamepad1);
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            double lT = leftOdometry.getInchesTravelled();
            double cT = centerOdometry.getInchesTravelled();
            double rT = rightOdometry.getInchesTravelled();

            double distanceRotated = (lT - rT) / 2;

            totalDistanceRotated += distanceRotated;
            totalMeasuredX += cT;

            telemetry.addData("totalDistanceRotated", totalDistanceRotated);
            telemetry.addData("totalMeasuredX", totalMeasuredX);
            telemetry.update();
        }
    }
}
