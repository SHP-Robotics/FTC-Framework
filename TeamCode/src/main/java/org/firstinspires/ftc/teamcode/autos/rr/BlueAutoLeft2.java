package org.firstinspires.ftc.teamcode.autos.rr;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.AdjustHolder;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.PracticeArmServo;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "ATestTeleOp")
public class BlueAutoLeft2 extends LinearOpMode {
    public enum State {
        LOCATION_1,
        DEPOSIT_1,
        TO_BACKDROP_1,

        LOCATION_2,
        DEPOSIT_2,
        TO_BACKDROP_2,

        LOCATION_3,
        LOCATION_3_POS,
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
        PracticeArmServo elbow = new PracticeArmServo(hardwareMap);
        PixelServo pixelServo = new PixelServo(hardwareMap);
        AdjustHolder wrist = new AdjustHolder(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        CommandScheduler myCommand = CommandScheduler.getInstance();

        //vision
        //TODO: SWITCH PIPELINE LATER
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap,"blue");
        location = visionSubsystem.getLocationRed();

        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
            location = visionSubsystem.getLocationBlue();
            telemetry.addLine("Trajectory Sequence Ready");
            telemetry.addData("Location: ", location);
            telemetry.update();
            telemetry.addData("arm state", arm.getState());
            arm.nextState();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (location) {
            case 1:
                Trajectory pixelToSpikeMarkOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28.75, -5, Math.toRadians(90)))
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
                        .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkThree);
                currentState = State.LOCATION_3_POS; //TODO: FOR TESTING
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
                                .lineToLinearHeading(new Pose2d(-32, 4, Math.toRadians(90)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkOneBackingUp);
                        currentState = State.IDLE;
                    }
                    break;
                case TO_BACKDROP_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkOneToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-20, -32, Math.toRadians(90)))
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
                        currentState = State.IDLE;
                    }
                    break;
                case TO_BACKDROP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-28, -32, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);

                        currentState = State.TO_PARKING;
                    }
                    break;
                case LOCATION_3_POS:
                if (!sampleMecanumDrive.isBusy()) {

                    Trajectory pixelAllTheWay = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-28.75, 6.5, Math.toRadians(-90)))
                            .build();
                    sampleMecanumDrive.followTrajectoryAsync(pixelAllTheWay);
                    currentState = State.DEPOSIT_3;
                }
                break;

                // Path series 3
                case DEPOSIT_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);
                        currentState = State.IDLE;
                    }
                    break;

                case TO_BACKDROP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkThreeToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-42, 32, Math.toRadians(90)))
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
                                .lineToLinearHeading(new Pose2d(-2,-37,Math.toRadians(90)))
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