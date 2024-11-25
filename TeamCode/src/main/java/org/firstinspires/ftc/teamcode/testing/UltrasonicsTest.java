package org.firstinspires.ftc.teamcode.testing;

import static org.apache.commons.math3.stat.StatUtils.min;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.shprobotics.pestocore.algorithms.MonteCarlo;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.sensors.DualUltrasonics;

import org.apache.commons.math3.analysis.function.Gaussian;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@Config
@TeleOp
public class UltrasonicsTest extends LinearOpMode {
    public static double MEAN = 0.0;
    public static double SIGMA = 0.1;

    @Override
    public void runOpMode() {
        AnalogInput ultrasonicLeft = (AnalogInput) hardwareMap.get("ultrasonicRight");
        AnalogInput ultrasonicRight = (AnalogInput) hardwareMap.get("ultrasonicLeft");

        Gaussian gaussian = new Gaussian(0, 1);
        DualUltrasonics dualUltrasonics = new DualUltrasonics(
                (pose2D, distance, heading) -> {
                    double distErr = min(new double[]{
                            abs(distance - pose2D.getX()),
                            abs(distance - pose2D.getY()),
                            abs(distance - pose2D.getX() + 144),
                            abs(distance - pose2D.getY() + 144)
                    });

                    double headingErr = abs(heading - pose2D.getHeadingRadians());

                    return gaussian.value(distErr) * gaussian.value(headingErr);
                },
        7.48
        );

        MonteCarlo monteCarlo = new MonteCarlo.MonteCarloBuilder(new Pose2D(0, 0, 0), 10).build();

        DeterministicTracker tracker = PestoFTCConfig.getTracker(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            tracker.update();

            telemetry.addData("ultrasonicLeft", ultrasonicLeft.getVoltage() * 62.039);
            telemetry.addData("ultrasonicRight", ultrasonicRight.getVoltage() * 62.039);

            dualUltrasonics.update(
                    ultrasonicLeft.getVoltage() * 62.039,
                    ultrasonicRight.getVoltage() * 62.039
            );

            monteCarlo.update(new Pose2D(0, 0, 0), dualUltrasonics);
            MonteCarlo.Particle particle = monteCarlo.getMostConfident();

            telemetry.addData("x", particle.getPose().getX());
            telemetry.addData("y", particle.getPose().getY());
            telemetry.addData("r", particle.getPose().getHeadingRadians());
            telemetry.addData("c", particle.getWeight());
            telemetry.update();
        }
    }
}
