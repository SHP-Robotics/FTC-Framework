package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kWristDown;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kWristDrive;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kWristName;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kLeftClawClosed;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kLeftClawName;
import static org.firstinspires.ftc.teamcode.Constants.Claw.kLeftClawOpen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous()
public class UltraBackupAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        Servo wrist = hardwareMap.get(Servo.class, kWristName);
        Servo leftClaw = hardwareMap.get(Servo.class, kLeftClawName);

        leftClaw.setPosition(kLeftClawClosed);
        sleep(2000);
        wrist.setPosition(kWristDrive);
        Trajectory ultraBackup = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(0)))
                .build();
        Trajectory Backup = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(5, 0, Math.toRadians(0)))
                .build();

        waitForStart();


        sampleMecanumDrive.followTrajectory(ultraBackup);
        wrist.setPosition(kWristDown);
        leftClaw.setPosition(kLeftClawOpen);
        sleep(2000);
        sampleMecanumDrive.followTrajectory(Backup);



    }
}
