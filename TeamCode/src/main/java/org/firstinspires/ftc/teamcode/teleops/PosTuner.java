package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PosTuner extends LinearOpMode {
    // spammable code
//    private boolean holdingUp = false;
//    private boolean holdingRight = false;
//    private boolean holdingDown = false;
//
//    private boolean changingWrist = true;
//
    Servo servo;

    @Override
    public void runOpMode() {
        servo = (Servo) hardwareMap.get("dropdown");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //        if (gamepad1.dpad_up) {
//            if (!holdingUp) {
//                if (changingWrist) {
//                    Constants.Intake.kWristDown += 0.005;
//                } else {
//                    Constants.Intake.kPositionBottom += 0.005;
//                }
//            }
//            holdingUp = true;
//        } else {
//            holdingUp = false;
//        }
//
//        if (gamepad1.dpad_down) {
//            if (!holdingDown) {
//                if (changingWrist) {
//                    Constants.Intake.kWristDown -= 0.005;
//                } else {
//                    Constants.Intake.kPositionBottom -= 0.005;
//                }
//            }
//            holdingDown = true;
//        } else {
//            holdingDown = false;
//        }
//
//        if (gamepad1.dpad_right) {
//            if (!holdingRight) {
//                changingWrist = !changingWrist;
//            }
//            holdingRight = true;
//        } else {
//            holdingRight = false;
//        }
//
//        telemetry.addData("kWristDown", Constants.Intake.kWristDown);
//        telemetry.addData("kPositionBottom", Constants.Intake.kPositionBottom);

            servo.setPosition(gamepad1.left_stick_y);
            telemetry.addData("pos", servo.getPosition());
            telemetry.update();
        }
    }
}
