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
//public class TestAuto extends BaseAuto {
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
