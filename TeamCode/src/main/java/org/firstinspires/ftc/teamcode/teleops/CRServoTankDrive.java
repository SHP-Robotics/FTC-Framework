package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class CRServoTankDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo left = (CRServo) hardwareMap.get("left");
        left.setDirection(DcMotorSimple.Direction.FORWARD);

        CRServo right = (CRServo) hardwareMap.get("right");
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            left.setPower(gamepad1.left_stick_y);
            right.setPower(gamepad1.right_stick_y);
        }
    }
}
