package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class BrakingCustom extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = (DcMotorEx) hardwareMap.get("motor");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0)
                motor.setVelocity(-gamepad1.left_trigger * motor.getVelocity());
            else
                motor.setPower(1);
        }
    }
}
