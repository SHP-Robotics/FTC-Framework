package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class CRServoTankDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo left = (CRServo) hardwareMap.get("left");
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo right = (CRServo) hardwareMap.get("right");
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        Servo leftPlow = (Servo) hardwareMap.get("leftPlow");
        Servo rightPlow = (Servo) hardwareMap.get("rightPlow");


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(-gamepad1.right_stick_y);
            leftPlow.setPosition(gamepad1.left_trigger);
            rightPlow.setPosition(-gamepad1.left_trigger);
        }
    }
}
