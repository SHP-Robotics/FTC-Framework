package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@Autonomous(preselectTeleOp = "Centerstage Field Oriented")
public class RedFarSide extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        this.side = Side.RED;

        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);
        MecanumController mecanumController = new MecanumController(hardwareMap);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);

        PurePursuitPath path;
        PurePursuitPath path1 = new PurePursuitPath.PurePursuitPathBuilder()
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_CLOSE);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(0, 30, Math.toRadians(90)))
                .rotateTo(new Position2D(0, 30, Math.toRadians(90)), Math.toRadians(180))
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .moveTo(new Position2D(48+60, 38, Math.toRadians(180)))
                .addAction(7, () -> {
                    mecanumController.deactivate();
                    outtake.setPosition(Constants.OUTTAKE_LOWERED);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(44+60, 38, Math.toRadians(180)))
                .addAction(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .moveTo(new Position2D(44+60, 10, Math.toRadians(180)))
                .moveTo(new Position2D(52+60, 10, Math.toRadians(180)))
                .build();
        PurePursuitPath path2 = new PurePursuitPath.PurePursuitPathBuilder()
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_CLOSE);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(5, 37.5, Math.toRadians(90)))
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .moveTo(new Position2D(5, 29.5, Math.toRadians(90)))
                .rotateTo(new Position2D(5, 29.5, Math.toRadians(90)), Math.toRadians(180))
                .moveTo(new Position2D(5, 12, Math.toRadians(180)))
                .moveTo(new Position2D(46+48, 12, Math.toRadians(180)))
                .moveTo(new Position2D(46+48, 34, Math.toRadians(180)))
                .moveTo(new Position2D(48+48, 34, Math.toRadians(180)))
                .addAction(7, () -> {
                    mecanumController.deactivate();
                    outtake.setPosition(Constants.OUTTAKE_LOWERED);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(44+48, 34.5, Math.toRadians(180)))
                .addAction(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .moveTo(new Position2D(44+48, 10, Math.toRadians(180)))
                .moveTo(new Position2D(52+48, 10, Math.toRadians(180)))
                .build();
        PurePursuitPath path3 = new PurePursuitPath.PurePursuitPathBuilder()
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_CLOSE);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(23.5, 28, Math.toRadians(90)))
                .rotateTo(new Position2D(23.5, 28, Math.toRadians(90)), Math.toRadians(180))
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .moveTo(new Position2D(23.5, 4, Math.toRadians(180)))
                .moveTo(new Position2D(49+60, 4, Math.toRadians(180)))
                .moveTo(new Position2D(49+60, 28, Math.toRadians(180)))
                .addAction(7, () -> {
                    mecanumController.deactivate();
                    outtake.setPosition(Constants.OUTTAKE_LOWERED);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(44+60, 28, Math.toRadians(180)))
                .addAction(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .moveTo(new Position2D(44+60, 10, Math.toRadians(180)))
                .moveTo(new Position2D(52+60, 10, Math.toRadians(180)))
                .build();

        super.runOpMode();
        switch (this.location) {
            case FAR:
                path = path1;
                break;
            case CLOSE:
                path = path3;
                break;
            default:
                path = path2;
                break;
        }
        path.followAsync(purePursuitFollower, mecanumController);

        while (opModeIsActive() && !isStopRequested()) {
            path.update();

            telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
            telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
            telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }

        mecanumController.deactivate();
    }
}
