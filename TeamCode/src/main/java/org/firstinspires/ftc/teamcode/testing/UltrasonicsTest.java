package org.firstinspires.ftc.teamcode.testing;

import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@TeleOp
public class UltrasonicsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput ultrasonicLeft = (AnalogInput) hardwareMap.get("ultrasonicRight");
        AnalogInput ultrasonicRight = (AnalogInput) hardwareMap.get("ultrasonicLeft");

        ColorSensor colorLeft = (ColorSensor) hardwareMap.get("colorLeft");
        ColorSensor colorRight = (ColorSensor) hardwareMap.get("colorRight");

        PID ultrasonicPID = new PID(0.02, 0.02, 0);
        ultrasonicPID.setMaxIntegralProportionRatio(0.24);
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        DeterministicTracker tracker = PestoFTCConfig.getTracker(hardwareMap);
        PathContainer pathContainer = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(
                        new Vector2D[]{
                                new Vector2D(0, 0),
                                new Vector2D(0, -40),
                        }
                ))
                .build();

        PathFollower pathFollower = PestoFTCConfig.generatePathFollower(pathContainer, mecanumController, tracker)
                .setCheckFinishedFunction((pathFollower1, toleranceXY, toleranceR) -> colorLeft.alpha() > 100 && colorRight.alpha() > 100)
                .setDecelerationFunction((pathFollower1, rotate) -> {
                    double avg = (ultrasonicLeft.getVoltage() + ultrasonicRight.getVoltage()) / 2;
                    avg *= 62.039;
                    return new Vector2D(0, max(-ultrasonicPID.getOutput(avg, 0), -0.3));
                })

                .setSpeed(0.4)
                .build();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pathFollower.update();
            tracker.update();

            telemetry.update();
        }
    }
}
