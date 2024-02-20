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
//        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
                .moveTo(new Position2D(10, 10, Math.toRadians(90)))
                .moveTo(new Position2D(10, 0, Math.toRadians(90)))
                .moveTo(new Position2D(0, 0, Math.toRadians(90)))

                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
                .moveTo(new Position2D(10, 10, Math.toRadians(90)))
                .moveTo(new Position2D(10, 0, Math.toRadians(90)))
                .moveTo(new Position2D(0, 0, Math.toRadians(90)))

                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
                .moveTo(new Position2D(10, 10, Math.toRadians(90)))
                .moveTo(new Position2D(10, 0, Math.toRadians(90)))
                .moveTo(new Position2D(0, 0, Math.toRadians(90)))
                .build();

        waitForStart();

        path.followAsync(purePursuitFollower, mecanumController);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            path.update();
//            mecanumController.simulateEncoders(new Position2D(5, 0, 0));

            telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
            telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
            telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("finished", path.isFinished());
            telemetry.addData("failed", path.failed());
            telemetry.addLine();
            telemetry.addData("lf", mecanumController.leftFront.getPower());
            telemetry.addData("rf", mecanumController.rightFront.getPower());
            telemetry.addData("lr", mecanumController.leftRear.getPower());
            telemetry.addData("rr", mecanumController.rightRear.getPower());
            telemetry.addLine();
            telemetry.addData("lf", mecanumController.leftFront.lastVelocity);
            telemetry.addData("rf", mecanumController.rightFront.lastVelocity);
            telemetry.addData("lr", mecanumController.leftRear.lastVelocity);
            telemetry.addData("rr", mecanumController.rightRear.lastVelocity);
            telemetry.update();
        }
    }
}
