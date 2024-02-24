package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.SimulatedMecanumController;

//@Disabled
@Autonomous()
public class PurePursuitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);
        SimulatedMecanumController mecanumController = new SimulatedMecanumController(hardwareMap);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
                .build();

        waitForStart();

        path.followAsync(purePursuitFollower, mecanumController);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            path.update();

            telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
            telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
            telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("finished", path.isFinished());
            telemetry.addData("failed", path.failed());
            telemetry.addLine();
            telemetry.addData("lf", mecanumController.motors[0].getPower());
            telemetry.addData("rf", mecanumController.motors[1].getPower());
            telemetry.addData("lr", mecanumController.motors[2].getPower());
            telemetry.addData("rr", mecanumController.motors[3].getPower());
            telemetry.addLine();
            telemetry.addData("lf", mecanumController.motors[0].getVelocity());
            telemetry.addData("rf", mecanumController.motors[1].getVelocity());
            telemetry.addData("lr", mecanumController.motors[2].getVelocity());
            telemetry.addData("rr", mecanumController.motors[3].getVelocity());
            telemetry.update();
        }
    }
}
