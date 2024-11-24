package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.shprobotics.pestocore.algorithms.MonteCarlo;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.sensors.DualUltrasonics;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@TeleOp
public class UltrasonicsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput ultrasonicLeft = (AnalogInput) hardwareMap.get("ultrasonicRight");
        AnalogInput ultrasonicRight = (AnalogInput) hardwareMap.get("ultrasonicLeft");

        DualUltrasonics dualUltrasonics = new DualUltrasonics(
                (double distance, double heading) -> {
                    return new Pose2D(distance, 0, heading);
                },
                7.48
        );

        MonteCarlo monteCarlo = new MonteCarlo(10);

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

            monteCarlo.update(tracker.getDeltaPosition(), dualUltrasonics);

            telemetry.update();
        }
    }
}
