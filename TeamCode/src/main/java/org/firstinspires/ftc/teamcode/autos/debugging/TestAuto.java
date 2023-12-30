package org.firstinspires.ftc.teamcode.autos.debugging;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //MecanumController mecanumController = new MecanumController(hardwareMap);
        //mecanumController.setRotationSpeed(0.2);

        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        TrajectorySequence trajectorySequence = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .addTemporalMarker(0, () -> {
                    outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                })
//                .addTemporalMarker(2, () -> {})
                .waitSeconds(3)
                .forward(5)
                .build();

        waitForStart();

        sampleMecanumDrive.followTrajectorySequence(trajectorySequence);

        /*

        mecanumController.leftFront.setPower(1);
        sleep(2000);
        mecanumController.rightFront.setPower(1);
        sleep(2000);
        mecanumController.leftRear.setPower(1);
        sleep(2000);
        mecanumController.rightRear.setPower(1);
        sleep(2000);

         */
    }
}
