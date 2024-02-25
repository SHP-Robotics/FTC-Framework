package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropDownSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@Autonomous
public class FarRed3 extends BaseAuto {
    PurePursuitPath path1;
    PurePursuitPath path2;
    PurePursuitPath path3;

    PurePursuitPath path;

    PurePursuitFollower purePursuitFollower;
    MecanumController mecanumController;

    @Override
    public void init() {
        this.side = Side.RED;

        double tanhPace = 0.5;
        double minimumTanh = Constants.minimumTanh;
        double maximumTanh = 0.75;

        path1 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(10, -29, Math.toRadians(90)))
                .moveTo(new Position2D(10, -19, Math.toRadians(90)))
                .rotateTo(new Position2D(10, -19, Math.toRadians(90)), Math.toRadians(0))
                .moveTo(new Position2D(10, -19-5, Math.toRadians(0)))

                .addAction(() -> {
                    intake.setState(IntakeSubsystem.State.AUTO_INTAKE);
                    dropDown.setState(DropDownSubsystem.State.RAISED);
                })
                .moveTo(new Position2D(21, -19-5, Math.toRadians(0)))

                .moveTo(new Position2D(18-24, -19, Math.toRadians(0)))
                .addAction(() -> {
                    intake.crWheel.setPower(-0.5);
                })
                .moveTo(new Position2D(18-24, -21, Math.toRadians(0)))
                .addAction(() -> {
                    intake.crWheel.setPower(0.5);
                    intake.setState(IntakeSubsystem.State.REJECT_SLOW);
                })
                .moveTo(new Position2D(18-24, -19-26-2, Math.toRadians(0)))
                .addAction(() -> intake.setState(IntakeSubsystem.State.STILL))
                .moveTo(new Position2D(18-24-72, -19-26-2, Math.toRadians(0)))
                .addAction(() -> {
                    arm.setState(ArmSubsystem.State.EXTENDED);
                    arm.setSlidePos(450);
                    elbow.setState(ElbowSubsystem.State.UP);
                    wrist.setState(WristSubsystem.State.UP);
                })
                .moveTo(new Position2D(18-24-72, -28.62, Math.toRadians(0)))
                .moveTo(new Position2D(-92, -28.62, Math.toRadians(0)))

                .addAction(2, () -> {
                    mecanumController.deactivate();
                    intake.pixelServo.setPosition(0.5);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(1000);
                    intake.crWheel.setPower(-1.0);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    intake.crWheel.setPower(0);
                    intake.pixelServo.setPosition(0.9);
                    arm.setState(ArmSubsystem.State.BOTTOM);
                    elbow.setState(ElbowSubsystem.State.DOWN);
                    wrist.setState(WristSubsystem.State.HALFWAY);
                })
                .moveTo(new Position2D(-88, -28, Math.toRadians(0)))
                .moveTo(new Position2D(-88, -47, Math.toRadians(0)))
                .moveTo(new Position2D(-100, -50, Math.toRadians(0)))

                .enableRetrace()
                .enableTanh(tanhPace, minimumTanh, maximumTanh)
                .build();

        path2 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, -31.5, Math.toRadians(90)))
                .moveTo(new Position2D(0, -20, Math.toRadians(90)))
                .rotateTo(new Position2D(0, -20, Math.toRadians(90)), Math.toRadians(0))
//                .moveTo(new Position2D(0, -20, Math.toRadians(0)))

                .addAction(() -> {
                    intake.setState(IntakeSubsystem.State.AUTO_INTAKE);
                    dropDown.setState(DropDownSubsystem.State.RAISED);
                })
                .moveTo(new Position2D(21, -19-5, Math.toRadians(0)))

                .moveTo(new Position2D(18, -19-5, Math.toRadians(0)))
                .addAction(() -> {
                    intake.crWheel.setPower(-0.5);
                })
                .moveTo(new Position2D(18, -19, Math.toRadians(0)))
                .moveTo(new Position2D(18, -19-5, Math.toRadians(0)))

                .addAction(() -> {
                    intake.crWheel.setPower(0.5);
                    intake.setState(IntakeSubsystem.State.REJECT_SLOW);
                })
                .moveTo(new Position2D(18, -19-26-2, Math.toRadians(0)))

                .moveTo(new Position2D(18-24, -19-26-2, Math.toRadians(0)))
                .addAction(() -> intake.setState(IntakeSubsystem.State.STILL))
                .moveTo(new Position2D(18-24-72, -19-26-2, Math.toRadians(0)))
                .addAction(() -> {
                    arm.setState(ArmSubsystem.State.EXTENDED);
                    arm.setSlidePos(450);
                    elbow.setState(ElbowSubsystem.State.UP);
                    wrist.setState(WristSubsystem.State.UP);
                })
                .moveTo(new Position2D(18-24-72, -25+2, Math.toRadians(0)))
                .moveTo(new Position2D(-92, -25+2, Math.toRadians(0)))

                .addAction(2, () -> {
                    mecanumController.deactivate();
                    intake.pixelServo.setPosition(0.5);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(1000);
                    intake.crWheel.setPower(-1.0);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    intake.crWheel.setPower(0);
                    intake.pixelServo.setPosition(0.9);
                    arm.setState(ArmSubsystem.State.BOTTOM);
                    elbow.setState(ElbowSubsystem.State.DOWN);
                    wrist.setState(WristSubsystem.State.HALFWAY);
                })
                .moveTo(new Position2D(-88, -28, Math.toRadians(0)))
                .moveTo(new Position2D(-88, -46, Math.toRadians(0)))
                .moveTo(new Position2D(-100, -50, Math.toRadians(0)))

                .enableRetrace()
                .enableTanh(tanhPace, minimumTanh, maximumTanh)
                .build();

        path3 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, -25, Math.toRadians(90)))
                .rotateTo(new Position2D(0, -25, Math.toRadians(90)), Math.toRadians(0))
                .moveTo(new Position2D(-7, -23, Math.toRadians(0)))
//                .moveTo(new Position2D(0, -16, Math.toRadians(0)))

                .addAction(() -> {
                    intake.setState(IntakeSubsystem.State.AUTO_INTAKE);
                    dropDown.setState(DropDownSubsystem.State.RAISED);
                })
                .moveTo(new Position2D(21, -19-5, Math.toRadians(0)))

                .moveTo(new Position2D(18-24, -19, Math.toRadians(0)))
                .addAction(() -> {
                    intake.crWheel.setPower(-0.5);
                })
                .moveTo(new Position2D(18-24, -21, Math.toRadians(0)))
                .addAction(() -> {
                    intake.crWheel.setPower(0.5);
                    intake.setState(IntakeSubsystem.State.REJECT_SLOW);
                })
                .moveTo(new Position2D(18-24, -19-26-3, Math.toRadians(0)))
                .addAction(() -> intake.setState(IntakeSubsystem.State.STILL))
                .moveTo(new Position2D(18-24-72, -19-26-3, Math.toRadians(0)))
                .addAction(() -> {
                    arm.setState(ArmSubsystem.State.EXTENDED);
                    arm.setSlidePos(450);
                    elbow.setState(ElbowSubsystem.State.UP);
                    wrist.setState(WristSubsystem.State.UP);
                })
                .moveTo(new Position2D(18-24-72, -19+1.75, Math.toRadians(0)))
                .moveTo(new Position2D(-92, -19+1.75, Math.toRadians(0)))

                .addAction(2, () -> {
                    mecanumController.deactivate();
                    intake.pixelServo.setPosition(0.5);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    arm.setPosition(1000);
                    intake.crWheel.setPower(-1.0);
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    intake.crWheel.setPower(0);
                    intake.pixelServo.setPosition(0.9);
                    arm.setState(ArmSubsystem.State.BOTTOM);
                    elbow.setState(ElbowSubsystem.State.DOWN);
                    wrist.setState(WristSubsystem.State.HALFWAY);
                })
                .moveTo(new Position2D(-88, -28, Math.toRadians(0)))
                .moveTo(new Position2D(-88, -46, Math.toRadians(0)))
                .moveTo(new Position2D(-100, -50, Math.toRadians(0)))

                .enableRetrace()
                .enableTanh(tanhPace, minimumTanh, maximumTanh)
                .build();

        purePursuitFollower = new PurePursuitFollower(hardwareMap);
        mecanumController = new MecanumController(hardwareMap);

        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        if (this.getLocation() == 1) {
            path = path1;
        } else if (this.getLocation() == 2) {
            path = path2;
        } else {
            path = path3;
        }

        path.followAsync(purePursuitFollower, mecanumController);

        super.start();
    }

    @Override
    public void loop() {
        telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
        telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
        telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
        telemetry.addLine();
        telemetry.addData("finished", path.isFinished());
        telemetry.addData("failed", path.failed());
        telemetry.addLine();
        telemetry.addData("geometries size", path.geometries.size());
        telemetry.addData("current geometry", path.getCurrentGeometry());
        if (path.geometries.size() > path.getCurrentGeometry()) {
            telemetry.addData("", path.geometries.get(path.getCurrentGeometry()));
        }
        telemetry.addLine();

        path.update();

        super.loop();
    }
}
