package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.VelocityApproximator;
import org.firstinspires.ftc.teamcode.debug.SpeedController;

@TeleOp
public class PurePursuitMaxVelocityTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);

        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .setNaturalSpeed(1)
                .build();
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setSpeedController(speedController);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double maxVelo = 0;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            mecanumController.drive(gamepad1);
            purePursuitFollower.updateOdometry();

            double[] approximatedVelocities = VelocityApproximator.getVelocities(purePursuitFollower.getRobotVelocity(), 1);

            if (VelocityApproximator.getMaxVelocity(approximatedVelocities) > Math.abs(maxVelo)) {
                maxVelo = VelocityApproximator.getMaxVelocity(approximatedVelocities);
            }

            telemetry.addData("leftFront approximated velocity", approximatedVelocities[0]);
            telemetry.addData("rightFront approximated velocity", approximatedVelocities[1]);
            telemetry.addData("leftRear approximated velocity", approximatedVelocities[2]);
            telemetry.addData("rightRear approximated velocity", approximatedVelocities[3]);
            telemetry.addLine();
            telemetry.addData("max velocity", maxVelo);
            telemetry.update();
        }
    }
}
