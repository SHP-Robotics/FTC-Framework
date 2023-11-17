package org.firstinspires.ftc.teamcode.centerstage.experiments;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@TeleOp(name = "Super Macro Speed Control")
public class SuperMacroSpeedControl extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPosition = new Pose2d(0, 0, 0);
        TrajectorySequence trajectorySequence = sampleMecanumDrive.trajectorySequenceBuilder(startPosition)
                        .forward(24)
                        .back(24)
                        .build();

        waitForStart();

        while(opModeIsActive()) {
            if (!sampleMecanumDrive.isBusy()) {
                sampleMecanumDrive.followTrajectorySequenceAsync(trajectorySequence);
            }
            sampleMecanumDrive.setDrivePower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x));
        }
    }
}
