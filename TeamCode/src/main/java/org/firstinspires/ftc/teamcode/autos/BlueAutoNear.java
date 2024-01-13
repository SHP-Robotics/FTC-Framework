package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CommandBasedTeleOp")
public class BlueAutoNear extends LinearOpMode {
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
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);

        //vision
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap,"blue");
        location = visionSubsystem.getLocationBlue();

        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
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
                        arm.setState(ArmSubsystem.State.INTAKE);
                        sleep(500);
                        claw.openLeftClaw();

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

                        claw.close();
                        arm.setState(ArmSubsystem.State.DRIVE);
                        sleep(1000);

                        currentState = State.TO_BACKDROP_1;
                    }
                    break;
                case TO_BACKDROP_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkOneToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(12, 0, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(20, -32, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkOneToParking);
                        currentState = State.DEPOSIT_TO_BACKDROP;
                    }
                    break;

                // Path series 2
                case BACKUP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);

                        claw.close();
                        arm.setState(ArmSubsystem.State.DRIVE);
                        sleep(1000);

                        currentState = State.TO_BACKDROP_2;
                    }
                    break;
                case TO_BACKDROP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28, -32, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);

                        currentState = State.DEPOSIT_TO_BACKDROP;
                    }
                    break;

                // Path series 3
                case BACKUP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, 0, Math.toRadians(0)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);

                        claw.close();
                        arm.setState(ArmSubsystem.State.DRIVE);
                        sleep(1000);

                        currentState = State.TO_BACKDROP_3;
                    }
                    break;

                case TO_BACKDROP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkThreeToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(42, -32, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkThreeToBackdrop);

                        currentState = State.DEPOSIT_TO_BACKDROP;
                    }

                    break;

                // Common states
                case DEPOSIT_TO_BACKDROP:
                    if (!sampleMecanumDrive.isBusy()) {
                        arm.setState(ArmSubsystem.State.OUTTAKE);
                        sleep(1000);
                        claw.openRightClaw();
                        sleep(1000);
                        claw.closeRightClaw();
                        arm.setState(ArmSubsystem.State.DRIVE);
                        currentState = State.TO_PARKING;
                    }
                    break;
                case TO_PARKING:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence backdropToPark = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .forward(5)
                                .lineToLinearHeading(new Pose2d(40,-37,Math.toRadians(90)))
                                .back(5)
                                .build();
                        sampleMecanumDrive.followTrajectorySequenceAsync(backdropToPark);
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
        }
    }
}