package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;

@Autonomous()
public class PurePursuitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap);
        mecanumPurePursuitController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder(new Position2D(0, 0, 0))
                .moveTo(new Position2D(0, 4, 0))
                .addAction(() -> sleep(1000))
                .moveTo(new Position2D(4, 4, 0))
                .build();

        waitForStart();

        path.followAsync(mecanumPurePursuitController);

        waitForStart();

        while (opModeIsActive()) {
            path.update();

            telemetry.addData("x", mecanumPurePursuitController.getCurrentPosition().getX());
            telemetry.addData("y", mecanumPurePursuitController.getCurrentPosition().getY());
            telemetry.addData("r", mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
