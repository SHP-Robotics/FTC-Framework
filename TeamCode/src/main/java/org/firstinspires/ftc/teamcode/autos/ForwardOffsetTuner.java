package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class ForwardOffsetTuner extends LinearOpMode {
    public MecanumController mecanumController;
    public DeterministicTracker tracker;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);

        PathContainer pathContainer = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(0, 0)
                                }
                        ),
                        new ParametricHeading(new double[]{0, Math.PI, 2*Math.PI})
                )
                .build();

        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            pathFollower.update();
            tracker.update();
        }
    }
}
