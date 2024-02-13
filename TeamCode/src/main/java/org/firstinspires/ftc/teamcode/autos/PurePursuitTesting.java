package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.GeometricShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;

@Autonomous()
public class PurePursuitTesting extends LinearOpMode {
    private void decrypt(Telemetry telemetry, Position2D position2D) {
        if (position2D == null) {
            telemetry.addLine("position is null");
            telemetry.addLine();
        } else {
            telemetry.addData("position X", position2D.getX());
            telemetry.addData("position Y", position2D.getY());
            telemetry.addData("position heading", position2D.getHeadingRadians());
            telemetry.addLine();
        }
    }

    private void decrypt(Telemetry telemetry, GeometricShape geometricShape) {
        decrypt(telemetry, geometricShape.getEndpoint());
    }

    private void decrypt(Telemetry telemetry, GeometricShape geometricShape, RestrictedCircle followingCircle) {
        Position2D[] intersections = geometricShape.circleIntersections(followingCircle);
        for (Position2D position2D: intersections) {
            decrypt(telemetry, position2D);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .rotateTo(new Position2D(0, 0, 0), Math.toRadians(180))
                .setFollowRadius(2)
                .setPositionBuffer(0.05)
                .setRotationBuffer(Math.toRadians(5))
                .setSpeedMin(0.1)
                .setSpeedCap(0.1)
//                .setPace(0.005)
                .build();
         */

        /*
        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .turnTo(new Position2D(10, 10, Math.toRadians(90)))
                .setFollowRadius(2)
                .setPositionBuffer(0.05)
                .setRotationBuffer(Math.toRadians(5))
                .setSpeedMin(0.1)
                .setSpeedCap(0.1)
//                .setPace(0.005)
                .build();
         */


        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, 10, Math.toRadians(90)))
                .setFollowRadius(2)
                .setPositionBuffer(0.2)
                .setRotationBuffer(Math.toRadians(5))
                .setSpeedMin(0.2)
                .setSpeedCap(0.2)
//                .setPace(0.005)
                .build();


        /*
        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(10, 10, Math.toRadians(90)))
                .setFollowRadius(2)
                .setPositionBuffer(0.05)
                .setRotationBuffer(Math.toRadians(5))
                .setSpeedMin(0.1)
                .setSpeedCap(0.1)
//                .setPace(0.005)
                .build();
         */

        /*
        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(10, 0, Math.toRadians(90)))
                .setFollowRadius(2)
                .setPositionBuffer(0.05)
                .setRotationBuffer(Math.toRadians(5))
                .setSpeedMin(0.1)
                .setSpeedCap(0.1)
//                .setPace(0.005)
                .build();
         */

        waitForStart();

        path.followAsync(purePursuitFollower, mecanumController);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            path.update();

            RestrictedCircle followingCircle = new RestrictedCircle(1);
            followingCircle.setOffset(purePursuitFollower.getCurrentPosition());

            decrypt(telemetry, path.geometries.get(0).getEndpoint());
            decrypt(telemetry, path.geometries.get(0), followingCircle);

            telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
            telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
            telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("finished", path.isFinished());
            telemetry.addData("failed", path.failed());
            telemetry.update();
        }
    }
}
