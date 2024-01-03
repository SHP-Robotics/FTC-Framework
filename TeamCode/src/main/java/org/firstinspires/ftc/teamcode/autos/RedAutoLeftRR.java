package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.io.File;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class RedAutoLeftRR extends LinearOpMode {
    private String soundPath = "/sdcard/FIRST/blocks/sounds";
    private File soundFile = new File(soundPath + "/Holy Moley.wav");

    public enum State {
        FORWARD_1,
        TURNING_1,
        TO_LOCATION_1,
        TO_BACKDROP_1,

        TO_LOCATION_2,
        TURNING_2,
        BACKING_UP_2,
        TURNING_TO_DROP_PIXEL_2,
        TO_BACKDROP_2,

        FORWARD_3,
        TURNING_TO_DROP_PIXEL_3,
        TO_LOCATION_3,
        BACKING_UP_3,
        TURNING_TO_BACKDROP_3,
        TO_BACKDROP_3,

        TO_PARKING,

        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousStorage.autonomousType = AutonomousStorage.AutonomousType.RedAutoLeftRR;

        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        VisionSubsystem visionSubsystem;
        State currentState;
        int location;

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(Constants.CLAW_CLOSE);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        visionSubsystem = new VisionSubsystem(hardwareMap,"red");
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
                Trajectory forwardOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(28.75, 0, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(forwardOne);
                currentState = State.FORWARD_1;
                break;
            case 2:
                Trajectory pixelToSpikeMarkTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(44.5, 0, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkTwo);
                currentState = State.TO_LOCATION_2;
                break;
            default:
                Trajectory forwardThree = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(28.75, 0, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(forwardThree);
                currentState = State.FORWARD_3;
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            sampleMecanumDrive.update();

            telemetry.addData("Current State", currentState);
            telemetry.update();

            switch (currentState) {
                // Path series 1
                case FORWARD_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(90));

                        currentState = State.TURNING_1;
                    }
                    break;
                case TURNING_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory pixelToSpikeMarkOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, 8.5, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkOne);

                        currentState = State.TO_LOCATION_1;
                    }
                    break;
                case TO_LOCATION_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        claw.setPosition(Constants.CLAW_OPEN);

                        TrajectorySequence spikeMarkOneToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, 0, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(47, 0, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(47, -84, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(32, -84, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(32, -88, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkOneToBackdrop);

                        currentState = State.TO_BACKDROP_1;
                    }
                    break;
                case TO_BACKDROP_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(1000);

                        TrajectorySequence spikeMarkOneToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(32, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -95, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkOneToParking);

                        currentState = State.TO_PARKING;
                    }

                    break;

                // Path series 2
                case TO_LOCATION_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(180));

                        currentState = State.TURNING_2;
                    }
                    break;
                case TURNING_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        claw.setPosition(Constants.CLAW_OPEN);
                        outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                        sleep(200);

                        Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(47, 0, Math.toRadians(180)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);

                        currentState = State.BACKING_UP_2;
                    }
                case BACKING_UP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(-90));

                        currentState = State.TURNING_TO_DROP_PIXEL_2;
                    }
                    break;
                case TURNING_TO_DROP_PIXEL_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(47, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(28.75, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(28.75, -87, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);

                        currentState = State.TO_BACKDROP_2;
                    }
                    break;
                case TO_BACKDROP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(1000);

                        TrajectorySequence spikeMarkTwoToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -95, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToParking);

                        currentState = State.TO_PARKING;
                    }
                    break;

                // Path series 3
                case FORWARD_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(-90));

                        currentState = State.TURNING_TO_DROP_PIXEL_3;
                    }
                    break;
                case TURNING_TO_DROP_PIXEL_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory pixelToSpikeMarkThree = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, -8, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkThree);

                        currentState = State.TO_LOCATION_3;
                    }
                    break;
                case TO_LOCATION_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        claw.setPosition(Constants.CLAW_OPEN);

                        Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, 0, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);

                        currentState = State.BACKING_UP_3;
                    }
                    break;
                case BACKING_UP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(180));

                        currentState = State.TURNING_TO_BACKDROP_3;
                    }
                    break;
                case TURNING_TO_BACKDROP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkThreeToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75+19, 0, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(28.75+19, -84, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(32-14, -84, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(32-14, -89, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkThreeToBackdrop);

                        currentState = State.TO_BACKDROP_3;
                    }
                    break;
                case TO_BACKDROP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(1000);

                        TrajectorySequence spikeMarkThreeToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(32-14, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -80, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -95, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkThreeToParking);

                        currentState = State.TO_PARKING;
                    }

                    break;

                // Common states

                case TO_PARKING:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_HIDDEN);

                        currentState = State.IDLE;
                    }

                    break;

                case IDLE:
                    break;
            }
        }
    }
}
