//package org.firstinspires.ftc.teamcode.teleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.debug.MecanumController;
//import org.firstinspires.ftc.teamcode.debug.config.Constants;
//
////@Disabled
//@TeleOp
//public class DriveBasic extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        MecanumController mecanumController = new MecanumController(hardwareMap);
//
//        int editing = 0;
//
//        boolean holdingDpadUp = false;
//        boolean holdingDpadDown = false;
//        boolean holdingDpadLeft = false;
//        boolean holdingDpadRight = false;
//
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            mecanumController.drive(gamepad1);
//
//            telemetry.addData("leftFrontPower", Constants.leftFrontPower);
//            telemetry.addData("rightFrontPower", Constants.rightFrontPower);
//            telemetry.addData("leftRearPower", Constants.leftRearPower);
//            telemetry.addData("rightRearPower", Constants.rightRearPower);
//            telemetry.addLine();
//
//            switch (editing) {
//                case 0:
//                    telemetry.addLine("editing left front power");
//
//                    if (gamepad1.dpad_up && !holdingDpadUp) {
//                        Constants.leftFrontPower += 0.01;
//                    }
//
//                    if (gamepad1.dpad_down && !holdingDpadDown) {
//                        Constants.leftFrontPower -= 0.01;
//                    }
//
//                    break;
//                case 1:
//                    telemetry.addLine("editing right front power");
//
//                    if (gamepad1.dpad_up && !holdingDpadUp) {
//                        Constants.rightFrontPower += 0.01;
//                    }
//
//                    if (gamepad1.dpad_down && !holdingDpadDown) {
//                        Constants.rightFrontPower -= 0.01;
//                    }
//
//                    break;
//                case 2:
//                    telemetry.addLine("editing left rear power");
//
//                    if (gamepad1.dpad_up && !holdingDpadUp) {
//                        Constants.leftRearPower += 0.01;
//                    }
//
//                    if (gamepad1.dpad_down && !holdingDpadDown) {
//                        Constants.leftRearPower -= 0.01;
//                    }
//
//                    break;
//                case 3:
//                    telemetry.addLine("editing right rear power");
//
//                    if (gamepad1.dpad_up && !holdingDpadUp) {
//                        Constants.rightRearPower += 0.01;
//                    }
//
//                    if (gamepad1.dpad_down && !holdingDpadDown) {
//                        Constants.rightRearPower -= 0.01;
//                    }
//
//                    break;
//                default:
//                    telemetry.addLine("editing overflow error");
//            }
//
//            telemetry.update();
//
//            if (gamepad1.dpad_right && !holdingDpadRight) {
//                editing = (editing + 1) % 4;
//            }
//
//            if (gamepad1.dpad_left && !holdingDpadLeft) {
//                editing = (editing + 3) % 4;
//            }
//
//            holdingDpadUp = gamepad1.dpad_up;
//            holdingDpadDown = gamepad1.dpad_down;
//            holdingDpadLeft = gamepad1.dpad_left;
//            holdingDpadRight = gamepad1.dpad_right;
//        }
//    }
//}
