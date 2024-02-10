package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.GeometricShape;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.RestrictedCircle;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.MecanumPurePursuitController;
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
        MecanumPurePursuitController mecanumPurePursuitController = new MecanumPurePursuitController(hardwareMap);
        mecanumPurePursuitController.setMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PurePursuitPath path = new PurePursuitPath.PurePursuitPathBuilder(new Position2D(0, 0, 0))
                .moveTo(new Position2D(0, 10, 0))
//                .addAction(() -> sleep(1000))
//                .moveTo(new Position2D(4, 4, 0))
                .setFollowRadius(2)
                .setMinimumTanh(0.15)
                .setMaximumTanh(0.3)
                .setPositionBuffer(0.05)
                .build();

        waitForStart();

        path.followAsync(mecanumPurePursuitController);

        waitForStart();

        while (opModeIsActive()) {
            path.update();

            RestrictedCircle followingCircle = new RestrictedCircle(1);
            followingCircle.setOffset(mecanumPurePursuitController.getCurrentPosition());

            decrypt(telemetry, path.geometries.get(0).getEndpoint());
            decrypt(telemetry, path.geometries.get(0), followingCircle);

            telemetry.addData("x", mecanumPurePursuitController.getCurrentPosition().getX());
            telemetry.addData("y", mecanumPurePursuitController.getCurrentPosition().getY());
            telemetry.addData("r", mecanumPurePursuitController.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("finished", path.isFinished());
            telemetry.addData("failed", path.failed());
            telemetry.update();
        }
    }
}
