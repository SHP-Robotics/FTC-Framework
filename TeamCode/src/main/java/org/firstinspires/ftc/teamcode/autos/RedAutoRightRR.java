package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.shplib.vision.ElementDetectionPipelineBlue;
import org.firstinspires.ftc.teamcode.shplib.vision.ElementDetectionPipelineRed;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.io.File;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class RedAutoRightRR extends LinearOpMode {
    private final String soundPath = "/sdcard/FIRST/blocks/sounds";
    private final File soundFile = new File(soundPath + "/Holy Moley.wav");

    public enum State {
        GEN_RIGHT,
        GEN_FORWARD,
        GEN_TURN,
        UNKNOWN,

        TO_BACKDROP_1,

        FORWARD_2,
        BACKING_UP_2,
        TURNING_2,
        TO_BACKDROP_2,

        FORWARD_3,
        TO_BACKDROP_3,

        TO_PARKING,

        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        VisionSubsystem visionSubsystem;
        State currentState;
        ElementDetectionPipelineRed.LocationPosition location;

        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setDirection(Servo.Direction.REVERSE);
        cameraServo.setPosition(Constants.CameraMode.FACING_TEAM_PROP.getPosition());

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
            if (location == ElementDetectionPipelineRed.LocationPosition.RIGHT) {
                telemetry.addLine("Spike mark 2");
            } else {
                telemetry.addLine("Spike mark unknown");
            }
            telemetry.addData("height", visionSubsystem.detectorRed.getMaxHeightReadable());
            telemetry.addData("mass", visionSubsystem.detectorRed.totalValue);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (location) {
            case RIGHT:
                Trajectory forwardTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(35, 0, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(forwardTwo);
                currentState = State.FORWARD_2;
                break;
            default:
                Trajectory genRight = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, -8, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(genRight);
                currentState = State.GEN_RIGHT;
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            sampleMecanumDrive.update();

            telemetry.addData("Current State", currentState);
            telemetry.update();

            switch (currentState) {
                case GEN_RIGHT:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory genForward = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(30, -8, Math.toRadians(0)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(genForward);
                        currentState = State.GEN_FORWARD;
                    }
                    break;
                case GEN_FORWARD:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(90));

                        currentState = State.GEN_TURN;
                    }
                    break;
                case GEN_TURN:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory genBack = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(30, -14, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(genBack);

                        currentState = State.UNKNOWN;
                    }
                    break;
                case UNKNOWN:
                    if (!sampleMecanumDrive.isBusy()) {
                        sleep(200);

                        if (visionSubsystem.detectorRed.getMaxHeightReadable() < 325) {
                            claw.setPosition(Constants.CLAW_OPEN);

                            TrajectorySequence unknownToBackdropOne = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(30, -43, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(30, -43.5, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(19, -43, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(19, -44, Math.toRadians(90)))
                                    .build();

                            sampleMecanumDrive.followTrajectorySequenceAsync(unknownToBackdropOne);

                            currentState = State.TO_BACKDROP_1;
                        } else {
                            Trajectory unknownForwardThree = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(30, 7.25, Math.toRadians(90)))
                                    .build();

                            sampleMecanumDrive.followTrajectoryAsync(unknownForwardThree);

                            currentState = State.FORWARD_3;
                        }
                    }
                    break;

                // Path series 1

                case TO_BACKDROP_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(500);

                        TrajectorySequence backdropOneToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(19, -38, Math.toRadians(90)))
                                .addDisplacementMarker(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                                .lineToLinearHeading(new Pose2d(3, -38, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -45, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(backdropOneToParking);

                        currentState = State.TO_PARKING;
                    }
                    break;

                // Path series 2
                case FORWARD_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        claw.setPosition(Constants.CLAW_OPEN);
                        outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                        sleep(100);

                        Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, 0, Math.toRadians(0)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);

                        currentState = State.BACKING_UP_2;
                    }
                case BACKING_UP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(90));

                        currentState = State.TURNING_2;
                    }
                case TURNING_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28.75, -42, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(26, -42, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(27, -42, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(27, -45, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);

                        currentState = State.TO_BACKDROP_2;
                    }
                case TO_BACKDROP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(500);

                        TrajectorySequence spikeMarkTwoToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(27, -39, Math.toRadians(90)))
                                .addDisplacementMarker(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                                .lineToLinearHeading(new Pose2d(3, -39, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -50, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToParking);

                        currentState = State.TO_PARKING;
                    }
                    break;

                // Path series 3
                case FORWARD_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        claw.setPosition(Constants.CLAW_OPEN);
                        sleep(100);

                        TrajectorySequence forwardThreeToBackdropThree = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(30, -43, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(30, -44.5, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(31.5, -44.5, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(forwardThreeToBackdropThree);

                        currentState = State.TO_BACKDROP_3;
                    }
                    break;
                case TO_BACKDROP_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(500);

                        TrajectorySequence spikeMarkThreeToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(31.5, -37, Math.toRadians(90)))
                                .addDisplacementMarker(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                                .lineToLinearHeading(new Pose2d(3, -37, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(3, -50, Math.toRadians(90)))
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
