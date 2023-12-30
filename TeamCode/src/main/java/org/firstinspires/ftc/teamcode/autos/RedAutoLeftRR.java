package org.firstinspires.ftc.teamcode.autos;

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
        LOCATION_1_TO_BACKDROP,
        LOCATION_2_TO_BACKDROP,
        LOCATION_3_TO_BACKDROP,
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

        TrajectorySequence trajectorySequenceSpikePositionOneToBackDrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(10)
                .turn(Math.toRadians(90))
                .strafeRight(20.5)
                .forward(7.5)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .back(10)
                .strafeRight(17)
                .back(80)
                .strafeLeft(22)
                .back(5)
                .build();

        TrajectorySequence trajectorySequenceSpikePositionOneToParking = sampleMecanumDrive.trajectorySequenceBuilder(trajectorySequenceSpikePositionOneToBackDrop.end())
                .forward(3)
                .strafeLeft(28)
                .addDisplacementMarker(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .back(12)
                .build();

        TrajectorySequence trajectorySequenceSpikePositionTwoToBackDrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(10)
                .turn(Math.toRadians(180))
                .back(32.75)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .back(8)
                .turn(Math.toRadians(-90))
                .back(80)
                .strafeLeft(22)
                .back(9)
                .build();

        TrajectorySequence trajectorySequenceSpikePositionTwoToParking = sampleMecanumDrive.trajectorySequenceBuilder(trajectorySequenceSpikePositionTwoToBackDrop.end())
                .forward(3)
                .strafeLeft(26)
                .addDisplacementMarker(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .back(12)
                .build();

        TrajectorySequence trajectorySequenceSpikePositionThreeToBackDrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(10)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                })
                .forward(-10)
                .build();

        TrajectorySequence trajectorySequenceSpikePositionThreeToParking = sampleMecanumDrive.trajectorySequenceBuilder(trajectorySequenceSpikePositionOneToParking.end())
                .forward(10)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                })
                .forward(-10)
                .build();

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
                sampleMecanumDrive.followTrajectorySequenceAsync(trajectorySequenceSpikePositionOneToBackDrop);
                currentState = State.LOCATION_1_TO_BACKDROP;
                break;
            case 2:
                sampleMecanumDrive.followTrajectorySequenceAsync(trajectorySequenceSpikePositionTwoToBackDrop);
                currentState = State.LOCATION_2_TO_BACKDROP;
                break;
            default:
                sampleMecanumDrive.followTrajectorySequenceAsync(trajectorySequenceSpikePositionThreeToBackDrop);
                currentState = State.LOCATION_3_TO_BACKDROP;
                break;
        }

        while (opModeIsActive()) {
            switch (currentState) {
                case LOCATION_1_TO_BACKDROP:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(2000);
                        sampleMecanumDrive.followTrajectorySequenceAsync(trajectorySequenceSpikePositionOneToParking);
                        currentState = State.TO_PARKING;
                    }
                    break;
                case LOCATION_2_TO_BACKDROP:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(2000);
                        sampleMecanumDrive.followTrajectorySequenceAsync(trajectorySequenceSpikePositionTwoToParking);
                        currentState = State.TO_PARKING;
                    }
                    break;
                case LOCATION_3_TO_BACKDROP:
                    if (!sampleMecanumDrive.isBusy()) {
                        outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                        sleep(2000);
                        sampleMecanumDrive.followTrajectorySequenceAsync(trajectorySequenceSpikePositionThreeToParking);
                        currentState = State.TO_PARKING;
                    }
                    break;
                case TO_PARKING:
                    if (!sampleMecanumDrive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundFile);
                    sleep(1200);
                    break;
            }

            sampleMecanumDrive.update();
            PoseStorage.currentPose = sampleMecanumDrive.getPoseEstimate();
        }
    }
}
