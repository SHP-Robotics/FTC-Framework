package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive roadrunnerDrive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Pose2d startPos = new Pose2d(0, 0, 0);

        TrajectorySequence trajectorySequence = roadrunnerDrive.trajectorySequenceBuilder(startPos)
                .forward(1)
                .build();

        roadrunnerDrive.followTrajectorySequence(trajectorySequence);
    }
}
