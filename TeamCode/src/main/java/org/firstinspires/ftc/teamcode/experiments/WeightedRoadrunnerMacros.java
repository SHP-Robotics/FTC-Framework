package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.debug.ElevatedMecanumController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class WeightedRoadrunnerMacros extends LinearOpMode {
    SampleMecanumDrive roadrunnerDrive;
    ElevatedMecanumController elevatedMecanumController;

    public void macro() {
        // Any trajectory sequence should work with the override
        // TODO: Find out if roadrunner accepts this as a compatible override, or if PID loops implode
        // Only works with closed loop positioning I.E. dead wheels and/or drive encoders
        TrajectorySequence trajectorySequence = roadrunnerDrive.trajectorySequenceBuilder(roadrunnerDrive.getPoseEstimate())
                .forward(24)
                .build();

        roadrunnerDrive.followTrajectorySequence(trajectorySequence);

        while (roadrunnerDrive.isBusy()) {
            elevatedMecanumController.driveWeights(gamepad1);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
        elevatedMecanumController = new ElevatedMecanumController(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                macro();
            }

            elevatedMecanumController.driveWeights(gamepad1);
            elevatedMecanumController.fullThrottle();
        }

        elevatedMecanumController.deactivate();
    }
}
