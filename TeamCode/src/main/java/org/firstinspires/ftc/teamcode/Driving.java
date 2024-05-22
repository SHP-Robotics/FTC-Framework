package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Driving extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor right = (DcMotor)hardwareMap.get("right");
        DcMotor left = (DcMotor)hardwareMap.get("left");
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            right.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
            left.setPower(-gamepad1.left_stick_y+gamepad1.right_stick_x);
        }
    }
}
