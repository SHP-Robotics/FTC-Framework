package org.firstinspires.ftc.teamcode.centerstage;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        MecanumController mecanumController = new MecanumController(hardwareMap);
//        mecanumController.setDriveSpeed(0.2);
//
//        Servo outtake = hardwareMap.get(Servo.class, "outtake");
//        outtake.setDirection(Servo.Direction.REVERSE);
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, "red");
        int location = visionSubsystem.getLocationRed();
        telemetry.addData("Location", location);
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            location = visionSubsystem.getLocationRed();
            telemetry.addData("Location", location);
            telemetry.update();
        }

        waitForStart();

//        outtake.setPosition(0.4);
//        sleep(2000);
//
//        outtake.setPosition(0.7);
//        sleep(2000);
//
//        outtake.setPosition(1);
//        sleep(2000);
//
//        mecanumController.moveInches(48, 48, 48, 48, true);
//        mecanumController.moveInches(48, -48, -48, 48, true);
    }
}
