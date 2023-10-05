package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.generals.Suspension;

@TeleOp(name = "test")
public class test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Suspension suspension = new Suspension(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            suspension.setPower(gamepad1.right_stick_y);
        }
    }
}
