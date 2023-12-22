package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class BlueAutoLeftRR extends LinearOpMode {
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
                    // TODO: determine if necessary
                    sleep(2000);
                })
                .forward(-10)
                .build();

        TrajectorySequence trajectorySequenceFront = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(35)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    // TODO: determine if necessary
                    sleep(2000);
                })
                .forward(-7)
                .turn(Math.toRadians(90))
                .forward(-10)
                .build();

        TrajectorySequence trajectorySequenceRight = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                .forward(10)
                .addDisplacementMarker(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    // TODO: determine if necessary
                    sleep(2000);
                })
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
    }
}
