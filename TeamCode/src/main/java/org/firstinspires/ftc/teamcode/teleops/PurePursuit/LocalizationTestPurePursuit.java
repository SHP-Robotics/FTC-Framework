package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.VelocityApproximator;
import org.firstinspires.ftc.teamcode.debug.SpeedController;

@Photon
@TeleOp()
public class LocalizationTestPurePursuit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);

        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.4)
                .build();
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setSpeedController(speedController);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lastTime = -1;
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
            purePursuitFollower.updateOdometry();

            telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
            telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
            telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            if (lastTime != -1) {
                double[] approximatedVelocities = VelocityApproximator.getVelocities(purePursuitFollower.getRobotVelocity(), 1);

                telemetry.addData("leftFront approximated velocity", approximatedVelocities[0]);
                telemetry.addData("rightFront approximated velocity", approximatedVelocities[1]);
                telemetry.addData("leftRear approximated velocity", approximatedVelocities[2]);
                telemetry.addData("rightRear approximated velocity", approximatedVelocities[3]);
            }
            lastTime = elapsedTime.seconds();
            telemetry.update();
        }
    }
}
