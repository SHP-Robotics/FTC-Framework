package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "experimental")
public class SubwaySurfers extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable sampleMecanumDriveCancelable = new SampleMecanumDriveCancelable(hardwareMap);

        Servo claw = hardwareMap.get(Servo.class, "claw");

        telemetry.addLine("Hardware initialized");
        telemetry.update();

        ArrayList<Pose2d> autonomousStartPoints = new ArrayList<>();
        // Red Auto Left RR
        autonomousStartPoints.add(new Pose2d(0, 0, 0));

        // Gets the position of the last run autonomous
        Pose2d startAutoPose = autonomousStartPoints.get(AutonomousStorage.autonomousType.ordinal());
        Pose2d endAutoPoseRel = PoseStorage.currentPose;

        Pose2d currentPose = new Pose2d(
                startAutoPose.getX() + endAutoPoseRel.getX(),
                startAutoPose.getY() + endAutoPoseRel.getY(),
                startAutoPose.getHeading() + endAutoPoseRel.getHeading()
        );

        PoseStorage.currentPose = currentPose;

        // Trajectory building

        TrajectorySequence toPixel = sampleMecanumDriveCancelable.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .lineTo(new Vector2d(15, 0))
                .turn(Math.toRadians(135))
                .lineTo(new Vector2d(0, -17))
                .build();

        TrajectorySequence toBackDrop = sampleMecanumDriveCancelable.trajectorySequenceBuilder(toPixel.end())
                .splineTo(new Vector2d(26, 82), Math.toRadians(-90))
                .build();

        telemetry.addLine("Hardware initialized");
        telemetry.addLine("Trajectories built");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            sampleMecanumDriveCancelable.followTrajectorySequenceAsync(toPixel);

            while (opModeIsActive() && sampleMecanumDriveCancelable.isBusy()) {}
            if (!opModeIsActive()) {
                sampleMecanumDriveCancelable.breakFollowing();
                return;
            }

            claw.setPosition(Constants.CLAW_CLOSE);
            sleep(2000);

            sampleMecanumDriveCancelable.followTrajectorySequenceAsync(toBackDrop);

            while (opModeIsActive() && sampleMecanumDriveCancelable.isBusy()) {}
            if (!opModeIsActive()) {
                sampleMecanumDriveCancelable.breakFollowing();
                return;
            }

            claw.setPosition(Constants.CLAW_OPEN);
            sleep(2000);
        }
    }
}
