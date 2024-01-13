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
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous()
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);

        waitForStart();

        // front left -> back left
        // front right -> back right
        // back left -> front left
        // back right -> front right

        mecanumController.leftFront.setPower(0.3);
        sleep(1000);
        mecanumController.rightFront.setPower(0.3);
        sleep(1000);
        mecanumController.leftRear.setPower(0.3);
        sleep(1000);
        mecanumController.rightRear.setPower(0.3);
        sleep(1000);

    }
}
