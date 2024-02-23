package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.SimulatedMecanumController;

@Disabled
@Autonomous()
public class PurePursuitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);
        SimulatedMecanumController mecanumController = new SimulatedMecanumController(hardwareMap);
//        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
//                .moveTo(new Position2D(10, 10, Math.toRadians(90)))
//                .moveTo(new Position2D(10, 0, Math.toRadians(90)))
//                .moveTo(new Position2D(0, 0, Math.toRadians(90)))
//
//                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
//                .moveTo(new Position2D(10, 10, Math.toRadians(90)))
//                .moveTo(new Position2D(10, 0, Math.toRadians(90)))
//                .moveTo(new Position2D(0, 0, Math.toRadians(90)))
//
//                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
//                .moveTo(new Position2D(10, 10, Math.toRadians(90)))
//                .moveTo(new Position2D(10, 0, Math.toRadians(90)))
//                .moveTo(new Position2D(0, 0, Math.toRadians(90)))
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
            telemetry.addLine();
            telemetry.addData("bottlenecks", path.bottlenecks[0]);
            telemetry.addData("bottlenecks", path.bottlenecks[1]);
            telemetry.addData("bottlenecks", path.bottlenecks[2]);
            telemetry.addData("bottlenecks", path.bottlenecks[3]);
            telemetry.addLine();
            telemetry.addData("target", path.targetVelocities[0]);
            telemetry.addData("target", path.targetVelocities[1]);
            telemetry.addData("target", path.targetVelocities[2]);
            telemetry.addData("target", path.targetVelocities[3]);
            telemetry.addLine();
            telemetry.addData("current", path.currentVelocities[0]);
            telemetry.addData("current", path.currentVelocities[1]);
            telemetry.addData("current", path.currentVelocities[2]);
            telemetry.addData("current", path.currentVelocities[3]);
            telemetry.update();
        }
    }
}
