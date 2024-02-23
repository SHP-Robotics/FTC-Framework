package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

@Autonomous(preselectTeleOp = "Centerstage Field Oriented")
public class Blue221 extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        this.side = Side.BLUE;

        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);
        MecanumController mecanumController = new MecanumController(hardwareMap);

        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(Constants.OUTTAKE_STARTING);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);

        PurePursuitPath path;
        PurePursuitPath pathFar = new PurePursuitPath.PurePursuitPathBuilder()
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_CLOSE);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(-0, 37, Math.toRadians(90)))
                .rotateTo(new Position2D(-0, 37, Math.toRadians(90)), Math.toRadians(180))
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .moveTo(new Position2D(-6, 37, Math.toRadians(180)))
                .moveTo(new Position2D(-6, 61, Math.toRadians(180)))
                .moveTo(new Position2D(-46-48, 61, Math.toRadians(180)))
                .moveTo(new Position2D(-46-48, 38, Math.toRadians(180)))
                .moveTo(new Position2D(-48-48, 38, Math.toRadians(180)))
                .addAction(7, () -> {
                    mecanumController.deactivate();
                    outtake.setPosition(Constants.OUTTAKE_LOWERED);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(-44-48, 38, Math.toRadians(180)))
                .addAction(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .moveTo(new Position2D(-44-48, 9, Math.toRadians(180)))
                .moveTo(new Position2D(-52-48, 9, Math.toRadians(180)))

                .enableRetrace()
                .build();
        PurePursuitPath pathCenter = new PurePursuitPath.PurePursuitPathBuilder()
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_CLOSE);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(-5, 37.5, Math.toRadians(90)))
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .moveTo(new Position2D(-5, 29.5, Math.toRadians(90)))
                .rotateTo(new Position2D(-5, 29.5, Math.toRadians(90)), Math.toRadians(180))
                .moveTo(new Position2D(-5, 61, Math.toRadians(180)))
                .moveTo(new Position2D(-46-48, 61, Math.toRadians(180)))
                .moveTo(new Position2D(-46-48, 37, Math.toRadians(180)))
                .moveTo(new Position2D(-48-48, 37, Math.toRadians(180)))
                .addAction(7, () -> {
                    mecanumController.deactivate();
                    outtake.setPosition(Constants.OUTTAKE_LOWERED);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(-44-48, 34.5, Math.toRadians(180)))
                .addAction(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .moveTo(new Position2D(-44-48, 9, Math.toRadians(180)))
                .moveTo(new Position2D(-52-48, 9, Math.toRadians(180)))

                .enableRetrace()
                .build();
        PurePursuitPath pathClose = new PurePursuitPath.PurePursuitPathBuilder()
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_CLOSE);
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(-0, 28, Math.toRadians(90)))
                .rotateTo(new Position2D(-0, 28, Math.toRadians(90)), Math.toRadians(0))
                .moveTo(new Position2D(-3, 28, Math.toRadians(0)))
                .addAction(() -> {
                    claw.setPosition(Constants.CLAW_OPEN);
                    outtake.setPosition(Constants.OUTTAKE_NEUTRAL);
                })
                .moveTo(new Position2D(5, 28, Math.toRadians(0)))
                .rotateTo(new Position2D(5, 28, Math.toRadians(0)), Math.toRadians(180))
                .moveTo(new Position2D(5, 61, Math.toRadians(180)))
                .moveTo(new Position2D(-46 - 48, 61, Math.toRadians(180)))
                .moveTo(new Position2D(-46 - 48, 33, Math.toRadians(180)))
                .moveTo(new Position2D(-49 - 48, 33, Math.toRadians(180)))
                .addAction(7, () -> {
                    mecanumController.deactivate();
                    outtake.setPosition(Constants.OUTTAKE_LOWERED);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .moveTo(new Position2D(-44 - 48, 33, Math.toRadians(180)))
                .addAction(() -> outtake.setPosition(Constants.OUTTAKE_HIDDEN))
                .moveTo(new Position2D(-44 - 48, 9, Math.toRadians(180)))
                .moveTo(new Position2D(-52 - 48, 9, Math.toRadians(180)))

                .enableRetrace()
                .build();

        super.runOpMode();
        switch (this.location) {
            case FAR:
                path = pathFar;
                break;
            case CLOSE:
                path = pathClose;
                break;
            default:
                path = pathCenter;
                break;
        }
        path.followAsync(purePursuitFollower, mecanumController);

        while (opModeIsActive() && !isStopRequested()) {
            path.update();

            telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
            telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
            telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("finished", path.isFinished());
            telemetry.addData("failed", path.failed());
            telemetry.addLine();
            telemetry.addData("power", mecanumController.motors[0].getPower());
            telemetry.addData("power", mecanumController.motors[1].getPower());
            telemetry.addData("power", mecanumController.motors[2].getPower());
            telemetry.addData("power", mecanumController.motors[3].getPower());
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

        mecanumController.deactivate();
    }
}
