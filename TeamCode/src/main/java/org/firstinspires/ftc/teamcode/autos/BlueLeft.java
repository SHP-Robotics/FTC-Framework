package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

//@Disabled
@Config
@Autonomous
public class BlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);

        PathContainer path = new PathContainer.PathContainerBuilder()
                .addCurve(
                        new BezierCurve(new Vector2D[]{
                                new Vector2D(0, 0),
                                new Vector2D(0, 12)
                        }),
                        () -> {
//                            slideSubsystem.setState(SlideSubsystem.SlideState.HIGH);
//                            intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTAKE);
//                            while (slideSubsystem.isBusy()) {}
//                            sleep(1000);
//                            intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
//                            slideSubsystem.setState(SlideSubsystem.SlideState.INTAKE);
                        }
                )
                .setIncrement(0.01)
                .build();

        PathFollower follower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                path)
                .setDeceleration(50)
                .setHeadingPID( new PID(0, 0, 0))
                .setEndpointPID(new PID(0, 0, 0))
                .setSpeed(0.4)
                .build();

        waitForStart();
        tracker.reset();

        while (opModeIsActive() && !isStopRequested() && !follower.isFinished(0.3)) {
            follower.update();
            tracker.updateOdometry();

            Vector2D currentPosition = tracker.getCurrentPosition();
            Pose2D currentVelocity = tracker.getRobotVelocity();
            telemetry.addData("X", currentPosition.getX());
            telemetry.addData("Y", currentPosition.getY());
            telemetry.addData("Rotation", tracker.getCurrentHeading());
            telemetry.addLine();
            telemetry.addData("X velocity", currentVelocity.getX());
            telemetry.addData("Y velocity", currentVelocity.getY());
            telemetry.addData("Rotational velocity", currentVelocity.getHeadingRadians());
            telemetry.update();
        }
    }
}
