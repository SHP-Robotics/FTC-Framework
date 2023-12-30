package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@TeleOp()
public class CancellableMacros extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTONOMOUS_INIT,
        AUTONOMOUS_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    MecanumController mecanumController;

    public void driveLoop() {
        while (opModeIsActive() && !isStopRequested() && !gamepad1.a) {
            mecanumController.drive(gamepad1);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        mecanumController = new MecanumController(hardwareMap);

        // We want to turn off velocity control for teleop
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: implement pose storage in autonomous
        // drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            switch (currentMode) {
                case AUTONOMOUS_INIT:
                    if (gamepad1.a) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    if (!drive.isBusy()) {
                        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(Math.sqrt(75)*2.375, 23.75/2, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(0, 23.75, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                                .build();

                        drive.followTrajectorySequenceAsync(traj2);

                        currentMode = Mode.AUTONOMOUS_CONTROL;
                    }
                    break;
                case AUTONOMOUS_CONTROL:
                    if (gamepad1.a) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    if (!drive.isBusy()) {
                        currentMode = Mode.AUTONOMOUS_INIT;
                    }
                    break;
            }
        }
    }
}