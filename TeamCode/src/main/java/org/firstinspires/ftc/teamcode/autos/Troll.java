package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous()
public class Troll extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo left = (CRServo) hardwareMap.get("left");
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo right = (CRServo) hardwareMap.get("right");
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        // Range 0-0.36666
        Servo leftPlow = (Servo) hardwareMap.get("leftPlow");
        // Range 0.622222-0
        Servo rightPlow = (Servo) hardwareMap.get("rightPlow");

        Servo eye = (Servo) hardwareMap.get("eye");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
//            left.setPower(-gamepad1.left_stick_y);
//            right.setPower(-gamepad1.right_stick_y);
//
//            if (gamepad1.square){
//                eye.setPosition(0.5);
//            }
//            if (gamepad1.triangle){
//                eye.setPosition(0.345);
//            }
//
//            if (gamepad1.circle){
//                eye.setPosition(0.25);
//            }
//
//            leftPlow.setPosition(gamepad1.left_trigger);
//            rightPlow.setPosition(-gamepad1.left_trigger);

            sleep(2000);
            eye.setPosition(0.5);
            leftPlow.setPosition(0.25);
            rightPlow.setPosition(1-0.25);

            sleep(6000);
            eye.setPosition(345);

            sleep(1510);
            eye.setPosition(0.5);

            sleep(1820);
            eye.setPosition(0.25);

            sleep(2000);
            eye.setPosition(0.345);
            leftPlow.setPosition(0);
            rightPlow.setPosition(1);

            sleep(1000);
            eye.setPosition(0.25);
            leftPlow.setPosition(0.366);
            rightPlow.setPosition(1-0.366);
        }
    }
}
