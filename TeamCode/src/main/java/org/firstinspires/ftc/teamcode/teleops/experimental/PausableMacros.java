package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

// 200+ lines of code to drive in a triangle during TeleOp
@TeleOp()
public class PausableMacros extends LinearOpMode {
    enum Mode {
        INIT,
        PATH_INIT,
        LINE_1,
        LINE_2,
        LINE_3
    }

    Mode currentMode = Mode.INIT;

    Pose2d path_init_end_position;
    Trajectory path_init;
    Pose2d line_1_end_position;
    Trajectory line_1;
    Pose2d line_2_end_position;
    Trajectory line_2;
    Pose2d line_3_end_position;
    Trajectory line_3;

    SampleMecanumDriveCancelable drive;
    MecanumController mecanumController;

    public void driveLoop() {
        while (opModeIsActive() && !isStopRequested() && !gamepad1.y) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            mecanumController.drive(gamepad1);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        mecanumController = new MecanumController(hardwareMap);

        // We want to turn off velocity control for teleop
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: implement pose storage in autonomous
        // drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        path_init_end_position = new Pose2d(0, 0, Math.toRadians(30));
        line_1_end_position = new Pose2d(Math.sqrt(75) * 2.375, 23.75 / 2, Math.toRadians(150));
        line_2_end_position = new Pose2d(0, 23.75, Math.toRadians(270));
        line_3_end_position = new Pose2d(0, 0, Math.toRadians(30));

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
                case INIT:
                    driveLoop();

                    if (drive.getPoseEstimate() != path_init_end_position) {
                        path_init = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(path_init_end_position)
                                .build();

                        drive.followTrajectoryAsync(path_init);
                    }

                    currentMode = Mode.PATH_INIT;

                    break;
                case PATH_INIT:
                    if (gamepad1.a) {
                        drive.breakFollowing();

                        driveLoop();

                        if (drive.getPoseEstimate() != path_init_end_position) {
                            path_init = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(path_init_end_position)
                                    .build();

                            drive.followTrajectoryAsync(path_init);
                        }
                    }

                    if (!drive.isBusy()) {
                        if (drive.getPoseEstimate() != line_1_end_position) {
                            line_1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(line_1_end_position)
                                    .build();

                            drive.followTrajectoryAsync(line_1);
                        }

                        currentMode = Mode.LINE_1;
                    }

                    break;

                case LINE_1:
                    if (gamepad1.a) {
                        drive.breakFollowing();

                        driveLoop();

                        if (drive.getPoseEstimate() != line_1_end_position) {
                            line_1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(line_1_end_position)
                                    .build();

                            drive.followTrajectoryAsync(line_1);
                        }
                    }

                    if (!drive.isBusy()) {
                        if (drive.getPoseEstimate() != line_2_end_position) {
                            line_2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(line_2_end_position)
                                    .build();

                            drive.followTrajectoryAsync(line_2);
                        }

                        currentMode = Mode.LINE_2;
                    }

                    break;

                case LINE_2:
                    if (gamepad1.a) {
                        drive.breakFollowing();

                        driveLoop();

                        if (drive.getPoseEstimate() != line_2_end_position) {
                            line_2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(line_2_end_position)
                                    .build();

                            drive.followTrajectoryAsync(line_2);
                        }
                    }

                    if (!drive.isBusy()) {
                        if (drive.getPoseEstimate() != line_3_end_position) {
                            line_3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(line_3_end_position)
                                    .build();

                            drive.followTrajectoryAsync(line_3);
                        }

                        currentMode = Mode.LINE_3;
                    }

                    break;

                case LINE_3:
                    if (gamepad1.a) {
                        drive.breakFollowing();

                        driveLoop();

                        if (drive.getPoseEstimate() != line_3_end_position) {
                            line_3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(line_3_end_position)
                                    .build();

                            drive.followTrajectoryAsync(line_3);
                        }
                    }

                    if (!drive.isBusy()) {
                        if (drive.getPoseEstimate() != path_init_end_position) {
                            path_init = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(path_init_end_position)
                                    .build();

                            drive.followTrajectoryAsync(path_init);
                        }

                        currentMode = Mode.PATH_INIT;
                    }

                    break;
            }
        }
    }
}