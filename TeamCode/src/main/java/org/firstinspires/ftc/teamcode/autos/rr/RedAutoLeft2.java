package org.firstinspires.ftc.teamcode.autos.rr;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

//@Autonomous(preselectTeleOp = "ATestTeleOp")
public class RedAutoLeft2 extends LinearOpMode {
    public enum State {
        LOCATION_1,
        DEPOSIT_1,
        TO_BACKDROP_1,

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

        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        State currentState;
        int location = 1;

        //subsystems
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        ElbowSubsystem elbow = new ElbowSubsystem(hardwareMap);
        PixelServo pixelServo = new PixelServo(hardwareMap);
        WristSubsystem wrist = new WristSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        //vision
        //TODO: SWITCH PIPELINE LATER
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap,"red");
        location = visionSubsystem.getLocationRed();

        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
            location = visionSubsystem.getLocationRed();
            telemetry.addLine("Trajectory Sequence Ready");
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (location) {
            case 1:
                Trajectory pixelToSpikeMarkOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28.75, -2, Math.toRadians(90)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkOne);
                currentState = State.DEPOSIT_1;
                break;
            case 2:
                Trajectory pixelToSpikeMarkTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-31, 0, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkTwo);
                currentState = State.DEPOSIT_2;
                break;
            default:
                Trajectory pixelToSpikeMarkThree = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28.75, 8.5, Math.toRadians(-90)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkThree);
                currentState = State.DEPOSIT_3; //TODO: FOR TESTING
                break;
        }
        //TODO: CURRENT AUTO 2+0 RIGHT RED
        while (opModeIsActive() && !isStopRequested()) {
            sampleMecanumDrive.update();

            telemetry.addData("Current State", currentState);
            telemetry.update();

            switch (currentState) {

                // Path series 1
                case DEPOSIT_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkOneBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-28.75, 4, Math.toRadians(90)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkOneBackingUp);
                        currentState = State.TO_BACKDROP_1;
                    }
                    break;
                case TO_BACKDROP_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkOneToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-3, 0, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-3, 75, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-40, 80, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkOneToParking);
                        currentState = State.TO_PARKING;
                    }
                    break;

                // Path series 2
                case DEPOSIT_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(0)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);
                        currentState = State.TO_BACKDROP_2;
                    }
                    break;
                case TO_BACKDROP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-3, 0, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-3, 75, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-28, 80, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);

                        currentState = State.TO_PARKING;
                    }
                    break;

                // Path series 3
                case DEPOSIT_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);
                        currentState = State.TO_BACKDROP_3;
                    }
                    break;

                case TO_BACKDROP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkThreeToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-3, 0, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-3, 75, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-25, 80, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkThreeToBackdrop);

                        currentState = State.TO_PARKING;
                    }

                    break;

                // Common states
                case DEPOSIT_TO_BACKDROP:
                    if (!sampleMecanumDrive.isBusy()) {
                        sleep(1000);
//                        new RaiseArmCommand(arm,wrist,elbow,pixelServo);
                        sleep(1000);
//                        pixelServo.setState(PixelServo.State.OUT);
                        sleep(1000);
//                        new LowerArmCommand(arm,wrist,elbow);
                        currentState = State.TO_PARKING;
                    }
                    break;
                case TO_PARKING:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence backdropToPark = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .forward(5)
                                .lineToLinearHeading(new Pose2d(-2,80,Math.toRadians(-90)))
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