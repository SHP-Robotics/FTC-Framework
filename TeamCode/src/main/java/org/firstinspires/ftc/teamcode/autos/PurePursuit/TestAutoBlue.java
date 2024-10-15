//package org.firstinspires.ftc.teamcode.autos.PurePursuit;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.debug.MecanumController;
//import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
//import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
//import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;
//import org.firstinspires.ftc.teamcode.debug.config.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//
//@Autonomous
//public class TestAutoBlue extends BaseAuto {
//    PurePursuitPath path1;
//    PurePursuitPath path2;
//    PurePursuitPath path3;
//
//    PurePursuitPath path;
//
//    PurePursuitFollower purePursuitFollower;
//    MecanumController mecanumController;
//
//    @Override
//    public void init() {
//        this.side = Side.BLUE;
//
//        path1 = new PurePursuitPath.PurePursuitPathBuilder()
//                .moveTo(new Position2D(8, -0, Math.toRadians(90)))
//                .moveTo(new Position2D(8, -27, Math.toRadians(90)))
//                .moveTo(new Position2D(8, -24, Math.toRadians(90)))
//
//                .enableRetrace()
//                .build();
//
//        path2 = new PurePursuitPath.PurePursuitPathBuilder()
//                .moveTo(new Position2D(-3, -31, Math.toRadians(90)), Constants.positionBuffer, Constants.rotationBuffer)
//                .moveTo(new Position2D(-3, -22, Math.toRadians(90)))
//                .rotateTo(new Position2D(-3, -22, Math.toRadians(90)), Math.toRadians(0))
//                .moveTo(new Position2D(-10, -20, Math.toRadians(0)))
//                .addAction(2, Math.toRadians(5), () -> {
//                    dropDown.setState(DropDownSubsystem.State.RAISED);
//                    arm.setState(ArmSubsystem.State.EXTENDED);
//                    arm.incrementState();
//                    elbow.setState(ElbowSubsystem.State.UP);
//                    wrist.setState(WristSubsystem.State.UP);
//                })
//                .moveTo(new Position2D(-44, -24, Math.toRadians(0)))
//                .addAction(3, () -> {
//                    mecanumController.deactivate();
//                    intake.crWheel.setPower(-1.0);
//                    intake.pixelServo.setPosition(0.5);
//                    try {
//                        Thread.sleep(2000);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                    intake.crWheel.setPower(0);
//                    intake.pixelServo.setPosition(0.9);
//                    arm.setState(ArmSubsystem.State.BOTTOM);
//                    elbow.setState(ElbowSubsystem.State.DOWN);
//                    wrist.setState(WristSubsystem.State.HALFWAY);
//                })
//                .moveTo(new Position2D(-0, -26, Math.toRadians(0)))
//                .addAction(() -> {
//                    wrist.setState(WristSubsystem.State.DOWN);
//                })
//                .moveTo(new Position2D(56, -26, Math.toRadians(0)))
//                .addAction(() -> {
//                    intake.setState(IntakeSubsystem.State.INTAKE);
//                })
//                .moveTo(new Position2D(65, -26, Math.toRadians(0)))
//                .addAction(0.3, Math.toRadians(5), () -> {
//                    mecanumController.deactivate();
//                    dropDown.setState(DropDownSubsystem.State.RAISED);
//                    try {
//                        Thread.sleep(100);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                    dropDown.setState(DropDownSubsystem.State.FOUR_HEIGHT);
//                    try {
//                        Thread.sleep(600);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                    dropDown.setState(DropDownSubsystem.State.GROUND_HEIGHT);
//                    try {
//                        Thread.sleep(600);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                })
//                .moveTo(new Position2D(20, -26, Math.toRadians(0)))
//                .addAction(2, Math.toRadians(5), () -> {
//                    intake.setState(IntakeSubsystem.State.STILL);
//                })
//                .moveTo(new Position2D(-10, -26, Math.toRadians(0)))
//                .addAction(2, Math.toRadians(5), () -> {
//                    arm.setState(ArmSubsystem.State.EXTENDED);
//                    arm.incrementState();
//                    elbow.setState(ElbowSubsystem.State.UP);
//                    wrist.setState(WristSubsystem.State.UP);
//                })
//                .moveTo(new Position2D(-48.7, -26, Math.toRadians(0)))
//                .addAction(2, () -> {
//                    mecanumController.deactivate();
//                    intake.crWheel.setPower(-1.0);
//                    intake.pixelServo.setPosition(0.5);
//                    try {
//                        Thread.sleep(2000);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                    intake.crWheel.setPower(0);
//                    intake.pixelServo.setPosition(0.9);
//                    arm.setState(ArmSubsystem.State.BOTTOM);
//                    elbow.setState(ElbowSubsystem.State.DOWN);
//                    wrist.setState(WristSubsystem.State.HALFWAY);
//                })
//
//                .enableRetrace()
//                .build();
//
//        path3 = new PurePursuitPath.PurePursuitPathBuilder()
//                .moveTo(new Position2D(-12, -0, Math.toRadians(90)))
//                .moveTo(new Position2D(-12, -27, Math.toRadians(90)))
//                .moveTo(new Position2D(-12, -24, Math.toRadians(90)))
//
//                .enableRetrace()
//                .build();
//
//        purePursuitFollower = new PurePursuitFollower(hardwareMap);
//        mecanumController = new MecanumController(hardwareMap);
//
//        super.init();
//    }
//
//    @Override
//    public void init_loop() {
//        super.init_loop();
//    }
//
//    @Override
//    public void start() {
//        if (this.getLocation() == 1) {
//            path = path1;
//        } else if (this.getLocation() == 2) {
//            path = path2;
//        } else {
//            path = path3;
//        }
//
//        path.followAsync(purePursuitFollower, mecanumController);
//
//        super.start();
//    }
//
//    @Override
//    public void loop() {
//        telemetry.addData("x", purePursuitFollower.getCurrentPosition().getX());
//        telemetry.addData("y", purePursuitFollower.getCurrentPosition().getY());
//        telemetry.addData("r", purePursuitFollower.getCurrentPosition().getHeadingRadians());
//        telemetry.addLine();
//        telemetry.addData("finished", path.isFinished());
//        telemetry.addData("failed", path.failed());
//        telemetry.addLine();
//        telemetry.addData("geometries size", path.geometries.size());
//        telemetry.addData("current geometry", path.getCurrentGeometry());
//        if (path.geometries.size() > path.getCurrentGeometry()) {
//            telemetry.addData("", path.geometries.get(path.getCurrentGeometry()));
//        }
//        telemetry.addLine();
//
//        path.update();
//
//        super.loop();
//    }
//}
