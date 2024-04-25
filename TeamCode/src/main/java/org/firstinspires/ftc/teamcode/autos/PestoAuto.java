package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.Tracker;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

import java.util.function.DoubleSupplier;

@TeleOp
public class PestoAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        DoubleSupplier timer = elapsedTime::seconds;

        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        PathContainer path = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(new Vector2D[]{
                        new Vector2D(0, 0),
                        new Vector2D(0, 24)
                }))
                .build();
        PathFollower follower = new PathFollower.PathFollowerBuilder(
                mecanumController,
                tracker,
                path,
                1,
                new PID(1, 0, 0, timer))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
        }
    }
}
