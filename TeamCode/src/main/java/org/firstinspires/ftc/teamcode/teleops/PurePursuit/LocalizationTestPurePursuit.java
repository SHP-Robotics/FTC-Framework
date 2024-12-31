//package org.firstinspires.ftc.teamcode.teleops.PurePursuit;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.shprobotics.pestocore.drivebases.MecanumController;
//import com.shprobotics.pestocore.drivebases.TeleOpController;
//import com.shprobotics.pestocore.drivebases.Tracker;
//import com.shprobotics.pestocore.geometries.Pose2D;
//import com.shprobotics.pestocore.geometries.Vector2D;
//
//import org.firstinspires.ftc.teamcode.PestoFTCConfig;
//
//@TeleOp()
//public class LocalizationTestPurePursuit extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
//        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
//        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
////            tracker.updateOdometry();
//
//            Vector2D pose = tracker.getCurrentPosition();
////            double heading = tracker.getCurrentHeading();
////            Pose2D velocity = tracker.getRobotVelocity();
//
//            telemetry.addData("x", pose.getX());
//            telemetry.addData("y", pose.getY());
//            telemetry.addData("r", heading);
//            telemetry.addLine();
//            telemetry.addData("Δx", velocity.getX());
//            telemetry.addData("Δy", velocity.getY());
//            telemetry.addData("Δr", velocity.getHeadingRadians());
//            telemetry.update();
//        }
//    }
//}
