package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.io.File;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class BlueAutoLeftRR extends LinearOpMode {
    private String soundPath = "/sdcard/FIRST/blocks/sounds";
    private File soundFile = new File(soundPath + "/Holy Moley.wav");

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        VisionSubsystem visionSubsystem;
        int location;

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(Constants.CLAW_CLOSE);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        TrajectorySequence trajectorySequenceLeft = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(10)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                })
                .waitSeconds(2)
                .forward(-10)
                .build();

        TrajectorySequence trajectorySequenceFront = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(35)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .waitSeconds(2)
                .forward(-9)
                .turn(Math.toRadians(90))
                .forward(-88)
                .addDisplacementMarker(() -> {
                    outtake.setPosition(Constants.OUTTAKE_ACTIVE);
                })
                .waitSeconds(2)
                .forward(3)
                .waitSeconds(1)
                .strafeLeft(26)
                .back(12)
                .build();

        TrajectorySequence trajectorySequenceRight = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(10)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                })
                .waitSeconds(2)
                .forward(-10)
                .build();

        visionSubsystem = new VisionSubsystem(hardwareMap,"blue");
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

        switch (location) {
            case 1:
                sampleMecanumDrive.followTrajectorySequence(trajectorySequenceLeft);
                break;
            case 2:
                sampleMecanumDrive.followTrajectorySequence(trajectorySequenceFront);
                break;
            case 3:
                sampleMecanumDrive.followTrajectorySequence(trajectorySequenceRight);
                break;
        }

        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundFile);
    }
}
