package org.firstinspires.ftc.teamcode.autos.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.vision.ElementDetectionPipelineBlue;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.io.File;

@Disabled
@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class BlueSpikeFarSide extends LinearOpMode {
    private final String soundPath = "/sdcard/FIRST/blocks/sounds";
    private final File soundFile = new File(soundPath + "/Holy Moley.wav");

    public enum State {
        GEN_FORWARD,
        GEN_TURN,
        UNKNOWN,

        TURN_TO_PIXEL_1,
        FORWARD_1,
        BACK_1,

        FORWARD_2,
        TURN_TO_PIXEL_2,
        BACKING_UP_2,

        FORWARD_3,

        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        VisionSubsystem visionSubsystem;
        State currentState;
        ElementDetectionPipelineBlue.LocationPosition location;

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(Constants.CLAW_CLOSE);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        visionSubsystem = new VisionSubsystem(hardwareMap,"blue");
        location = visionSubsystem.getLocationBlue();
        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
            location = visionSubsystem.getLocationBlue();
            telemetry.addLine("Trajectory Sequence Ready");
            telemetry.addData("Location: ", location);
            telemetry.addData("height", visionSubsystem.detectorBlue.getMaxHeightReadable());
            telemetry.addData("mass", visionSubsystem.detectorBlue.totalValue);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (location == ElementDetectionPipelineBlue.LocationPosition.RIGHT) {
            Trajectory forwardTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(46, 0, Math.toRadians(0)))
                    .build();

            sampleMecanumDrive.followTrajectoryAsync(forwardTwo);
            currentState = State.FORWARD_2;
        } else {
            Trajectory genForward = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(0)))
                    .build();

            sampleMecanumDrive.followTrajectoryAsync(genForward);
            currentState = State.GEN_FORWARD;
        }

        while (opModeIsActive() && !isStopRequested()) {
            sampleMecanumDrive.update();

            telemetry.addData("Current State", currentState);
            telemetry.addData("max height", visionSubsystem.detectorBlue.getMaxHeightReadable());
            telemetry.update();

            switch (currentState) {
                case GEN_FORWARD:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(-90));

                        currentState = State.GEN_TURN;
                    }
                    break;
                case GEN_TURN:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory genLeft = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28, 7, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(genLeft);

                        currentState = State.UNKNOWN;
                    }
                    break;
                case UNKNOWN:
                    if (!sampleMecanumDrive.isBusy()) {
                        sleep(500);

                        if (visionSubsystem.detectorBlue.getMaxHeightReadable() < 170) {
                            sampleMecanumDrive.turnAsync(Math.toRadians(180));

                            currentState = State.TURN_TO_PIXEL_1;
                        } else {
                            Trajectory unknownForwardThree = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(28, -6, Math.toRadians(-90)))
                                    .build();

                            sampleMecanumDrive.followTrajectoryAsync(unknownForwardThree);

                            currentState = State.FORWARD_3;
                        }
                    }
                    break;

                // Path series 1

                case TURN_TO_PIXEL_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                        sleep(2000);

                        Trajectory forwardOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28, 10, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(forwardOne);

                        currentState = State.FORWARD_1;
                    }
                    break;

                case FORWARD_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        claw.setPosition(Constants.CLAW_OPEN);
                        sleep(1000);

                        Trajectory backOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28, 0, Math.toRadians(90)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(backOne);

                        currentState = State.BACK_1;
                    }
                    break;

                case BACK_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(180));

                        currentState = State.IDLE;
                    }
                    break;


                // Path series 2
                case FORWARD_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(180));

                        currentState = State.TURN_TO_PIXEL_2;
                    }
                    break;
                case TURN_TO_PIXEL_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        claw.setPosition(Constants.CLAW_OPEN);

                        Trajectory backingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(180)))
                                .build();

                        sampleMecanumDrive.followTrajectoryAsync(backingUp);

                        currentState = State.BACKING_UP_2;
                    }
                    break;
                case BACKING_UP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        sampleMecanumDrive.turnAsync(Math.toRadians(90));

                        currentState = State.IDLE;
                    }
                    break;

                // Path series 3
                case FORWARD_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                        claw.setPosition(Constants.CLAW_OPEN);
                        sleep(1000);

                        currentState = State.IDLE;
                    }
                    break;

                // Common states

                case IDLE:
                    break;
            }
        }
    }
}
