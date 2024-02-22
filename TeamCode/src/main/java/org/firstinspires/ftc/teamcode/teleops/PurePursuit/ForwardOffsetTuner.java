package org.firstinspires.ftc.teamcode.teleops.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@TeleOp()
public class ForwardOffsetTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);

        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedController.SpeedType.NO_CHANGE)
                .setNaturalSpeed(0.4)
                .build();
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setSpeedController(speedController);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Constants.FORWARD_OFFSET = 0;

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            mecanumController.drive(gamepad1);
            purePursuitFollower.updateOdometry();
            telemetry.addLine("4. Complete OdometryWidthTuner");
            telemetry.addLine("5. Rotate 10 full times counter clockwise");
            telemetry.addLine("6. Press B");
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("7. Set forward offset to", -(purePursuitFollower.getCurrentPosition().dist(new Position2D(0, 0, 0))/(20*Math.PI)));
            telemetry.update();
        }
    }
}
