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
        MecanumController mecanumDrive = new MecanumController(hardwareMap);

        waitForStart();

        mecanumDrive.leftFront.setPower(1);
        sleep(1000);
        mecanumDrive.rightFront.setPower(1);
        sleep(1000);
        mecanumDrive.leftRear.setPower(1);
        sleep(1000);
        mecanumDrive.rightRear.setPower(1);
        sleep(1000);
    }
}
