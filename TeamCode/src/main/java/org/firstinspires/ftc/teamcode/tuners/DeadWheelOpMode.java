package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

import java.util.List;

@TeleOp
public class DeadWheelOpMode extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        ThreeWheelOdometryTracker threeWheelOdometryTracker = (ThreeWheelOdometryTracker) PestoFTCConfig.getTracker(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime();

        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module: modules)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module: modules)
                module.clearBulkCache();

            double ms = elapsedTime.seconds();

            telemetry.addData("left", threeWheelOdometryTracker.leftOdometry.getInchesTravelled() / ms);
            telemetry.addData("right", threeWheelOdometryTracker.rightOdometry.getInchesTravelled() / ms);
            telemetry.addData("center", threeWheelOdometryTracker.centerOdometry.getInchesTravelled() / ms);
            telemetry.update();

            elapsedTime.reset();
        }
    }
}
