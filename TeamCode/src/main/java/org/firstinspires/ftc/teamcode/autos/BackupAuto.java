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

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CommandBasedTeleOp")
public class BackupAuto extends LinearOpMode {
    public enum State {
        LOCATION_1,
        DEPOSIT,
        BACKUP_1,
        TO_BACKDROP_1,

        LOCATION_2,
        BACKUP_2,
        TO_BACKDROP_2,

        LOCATION_3,
        BACKUP_3,
        TO_BACKDROP_3,
        DEPOSIT_TO_BACKDROP,

        TO_PARKING,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        State currentState;
        int location = 1;

        //subsystems
        Servo wrist = hardwareMap.get(Servo.class, kWristName);
        Servo leftClaw = hardwareMap.get(Servo.class, kLeftClawName);

        //vision
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap,"blue");
        location = visionSubsystem.getLocationBlue();

        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
            leftClaw.setPosition(kLeftClawClosed);
            wrist.setPosition(kWristDrive);
            location = visionSubsystem.getLocationBlue();
            telemetry.addLine("Trajectory Sequence Ready");
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (location) {
            case 1:
                Trajectory pixelToSpikeMarkOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(28.75, -2, Math.toRadians(-90)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkOne);
                currentState = State.DEPOSIT;
                break;
            case 2:
                Trajectory pixelToSpikeMarkTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkTwo);
                currentState = State.DEPOSIT;
                break;
            default:
                Trajectory pixelToSpikeMarkThree = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(28.75, 8.5, Math.toRadians(90)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkThree);
                currentState = State.DEPOSIT; //TODO: FOR TESTING
                break;
        }
        //TODO: CURRENT AUTO 2+0 RIGHT RED
        while (opModeIsActive() && !isStopRequested()) {
            sampleMecanumDrive.update();

            telemetry.addData("Current State", currentState);
            telemetry.update();

            switch (currentState) {

                // Path series 1
                case DEPOSIT:
                    if (!sampleMecanumDrive.isBusy()) {
                        wrist.setPosition(kWristDown);
                        sleep(1000);
                        leftClaw.setPosition(kLeftClawOpen);;

                        if(location == 1)
                            currentState = State.BACKUP_1;
                        else if(location == 2)
                            currentState = State.BACKUP_2;
                        else
                            currentState = State.BACKUP_3;
                    }
                    break;
                case BACKUP_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkOneBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, 4, Math.toRadians(-90)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkOneBackingUp);

                        currentState = State.IDLE;
                    }
                    break;

                // Path series 2
                case BACKUP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);

                        currentState = State.IDLE;
                    }
                    break;

                // Path series 3
                case BACKUP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, 0, Math.toRadians(0)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);

                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    break;
            }
        }
    }
}