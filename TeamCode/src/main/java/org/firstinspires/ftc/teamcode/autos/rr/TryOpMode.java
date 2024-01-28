package org.firstinspires.ftc.teamcode.autos.rr;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kAdjustHolder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;

//@Autonomous(preselectTeleOp = "ATestTeleOp")
public class TryOpMode extends LinearOpMode {
    public enum State {
        LOCATION_1,
        DEPOSIT_1,
        TO_BACKDROP_1,
        ARM_1,

        LOCATION_2,
        DEPOSIT_2,
        TO_BACKDROP_2,

        LOCATION_3,
        DEPOSIT_3,
        TO_BACKDROP_3,
        DEPOSIT_TO_BACKDROP,

        TO_PARKING,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousStorage.autonomousType = AutonomousStorage.AutonomousType.RedAutoLeftRR;

        DcMotor leftSlide = hardwareMap.get(DcMotor.class, kLeftSlideName);
        Servo adjustHolder = hardwareMap.get(Servo.class, kAdjustHolder);

        //subsystems
        waitForStart();
        leftSlide.setPower(-0.5);
        sleep(500);
        adjustHolder.setPosition(0.2);
        //vision


    }
}