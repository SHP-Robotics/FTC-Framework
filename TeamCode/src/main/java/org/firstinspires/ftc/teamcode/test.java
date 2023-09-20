package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "test")
public class test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            lift.setPower(-gamepad1.right_stick_y);
        }
    }
}
