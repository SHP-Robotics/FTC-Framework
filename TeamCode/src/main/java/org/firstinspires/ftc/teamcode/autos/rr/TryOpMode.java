package org.firstinspires.ftc.teamcode.autos.rr;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kLeftSlideName;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
import org.firstinspires.ftc.teamcode.commands.LowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.subsystems.AdjustHolder;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.PracticeArmServo;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "ATestTeleOp")
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
        leftSlide.setPower(-0.5);
        //subsystems
        waitForStart();

        //vision

        sleep(10000);
    }
}