package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Tele Op")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "claw");

        while (opModeIsActive()) {
            claw.setPosition(gamepad1.left_stick_y);
            telemetry.addData("position", gamepad1.left_stick_y);
            telemetry.update();
            sleep(20);
        }
    }
}
